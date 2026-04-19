#!/usr/bin/env bash
#
# =============================================================================
# Web「仿真上线」唯一入口（本仓库 sim 栈）
# =============================================================================
# 在本脚本内**顺序后台启动**同一命名空间 ${RID} 下的：
#   1) simulate（Gazebo + 机器人）
#   2) heartbeat（RobotStatus 发布；Lifecycle 需 configure→activate）
#   3) manager（health_monitor + task_manager，依赖上一步的 ~/set_heartbeat_params 与 ~/robot_status）
#   4) slam_bringup localization + mapping（lifecycle 二选一 active，由 AUTO_MAPPING 决定）
#   5) nav_bringup stack
#
# **仿真下线**不再调用 sim_shutdown.sh：由上层将 ``RobotStatus.robot_status`` 置为 **SHUTDOWN**
#（`RobotStatus.msg` 中 ``ROBOT_STATUS_SHUTDOWN=4``），典型路径为 Web「仿真离线」→
# ``robot_lifecycle.shutdown_selected_robot`` → ``ros2 service call /<id>/set_heartbeat_params``；
# simulate / nav / slam 等应订阅 ``/<id>/robot_status`` 或监听该状态并自行 pkill 或 lifecycle 收尾
#（本脚本不负责杀进程）。
#
# 用法:
#   OPEN_DELIVERY_ROOT=/path/to/openDelivery SIM_MODE=sim sim_bringup.sh <robot_id>
# 可选环境变量:
#   SIM_BRINGUP_VERBOSE=1          打开 bash -x
#   SIM_BRINGUP_MANAGER_WAIT=2   heartbeat 就绪后等待秒数再拉 manager（默认 2）
#   SIM_BRINGUP_MANAGER_POSE_TOPIC=amcl_pose   传给 health_monitor；置空则关闭位姿判定 ready
#
# 勿用 set -u：/opt/ros/*/setup.bash 与 install/setup.bash 会引用未设置的变量（如 AMENT_TRACE_SETUP_FILES），
# 与 backend RosNodeManager._bash_prefix 一致，仅用 -e 与 pipefail。

set -eo pipefail

log() {
  echo "[sim_bringup $(date -Iseconds)] $*" >&2
}

cleanup() {
  local p
  for p in $(jobs -p 2>/dev/null); do
    kill "${p}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

RID="${1:?robot id required (e.g. robot2)}"
SIM_MODE="${SIM_MODE:-sim}"

if [[ "${SIM_BRINGUP_VERBOSE:-0}" == "1" ]]; then
  set -x
fi

ROOT="${OPEN_DELIVERY_ROOT:-}"
if [[ -z "${ROOT}" ]]; then
  _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  ROOT="$(cd "${_here}/../../../../" && pwd)"
fi

log "robot_id=${RID} SIM_MODE=${SIM_MODE} OPEN_DELIVERY_ROOT=${ROOT}"

ROS_DISTRO="${ROS_DISTRO:-foxy}"
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  log "ERROR: missing /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
log "ros2=$(command -v ros2 || echo MISSING)"

if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${ROOT}/install/setup.bash"
  log "sourced ${ROOT}/install/setup.bash"
else
  log "WARN: no ${ROOT}/install/setup.bash"
fi
cd "${ROOT}"

gazebo_running() {
  if pgrep -f gzserver >/dev/null 2>&1; then
    log "gzserver already running"
    return 0
  fi
  if timeout 4 ros2 service list 2>/dev/null | grep -qE '/gazebo/'; then
    log "/gazebo/ services present"
    return 0
  fi
  return 1
}

if gazebo_running; then
  START_GZ="false"
else
  START_GZ="true"
fi
log "simulate: start_gazebo:=${START_GZ} spawn_robot:=true"

STORE="${ROOT}/backend/data/robot_status_last.json"
AUTO_MAPPING=0
if [[ -f "${STORE}" ]]; then
  AUTO_MAPPING="$(python3 -c "
import json, sys
rid, path = sys.argv[1], sys.argv[2]
try:
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    e = d.get(rid) or {}
    s = str(e.get('task_status') or e.get('robot_status', '')).strip().lower()
    print(1 if s == 'mapping' else 0)
except Exception:
    print(0)
" "${RID}" "${STORE}")"
fi
log "AUTO_MAPPING=${AUTO_MAPPING} (from persisted robot_status in ${STORE})"

SIM_WAIT="${SIM_BRINGUP_SIM_WAIT:-6}"
HB_WAIT="${SIM_BRINGUP_HEARTBEAT_WAIT_SEC:-20}"
MANAGER_WAIT="${SIM_BRINGUP_MANAGER_WAIT:-2}"
MANAGER_POSE_TOPIC="${SIM_BRINGUP_MANAGER_POSE_TOPIC:-amcl_pose}"
NAV_WAIT="${SIM_BRINGUP_NAV_WAIT_SEC:-120}"
SLAM_WAIT="${SIM_BRINGUP_SLAM_WAIT_SEC:-30}"

NAV_GRID_MODE="${SIM_BRINGUP_NAV_GRID_MODE:-localize}"
if [[ "${AUTO_MAPPING}" == "1" ]]; then
  NAV_GRID_MODE="mapping"
fi
log "NAV_GRID_MODE=${NAV_GRID_MODE}"

# --- 1) Gazebo + 机器人 ---
ros2 launch simulate simulate.launch.py \
  "robot_name:=${RID}" \
  "namespace:=${RID}" \
  "start_gazebo:=${START_GZ}" \
  "spawn_robot:=true" \
  "use_sim_time:=true" &
log "started simulate.launch.py (pid $!)"
sleep "${SIM_WAIT}"

# --- 2) 心跳 ---
ros2 launch heartbeat heartbeat.launch.py \
  "namespace:=${RID}" \
  "robot_name:=${RID}" \
  "sim_mode:=${SIM_MODE}" &
log "started heartbeat.launch.py (pid $!)"
HB="/${RID}/heartbeat"
# heartbeat_node is still a LifecycleNode; wait briefly for service visibility then best-effort activate.
hb_ready=0
for _ in $(seq 1 "${HB_WAIT}"); do
  if ros2 lifecycle get "${HB}" >/dev/null 2>&1; then
    hb_ready=1
    break
  fi
  sleep 1
done
if [[ "${hb_ready}" == "1" ]]; then
  if ros2 lifecycle set "${HB}" configure >/dev/null 2>&1; then
    log "heartbeat configure requested: ${HB}"
  else
    log "heartbeat configure skipped/failed (may already be configured): ${HB}"
  fi
  if ros2 lifecycle set "${HB}" activate >/dev/null 2>&1; then
    log "heartbeat activate requested: ${HB}"
  else
    log "heartbeat activate skipped/failed (may already be active): ${HB}"
  fi
else
  log "WARN: heartbeat lifecycle service not visible within ${HB_WAIT}s: ${HB}"
fi

# --- 2b) manager：health_monitor + task_manager（包名 manager，与 heartbeat 同命名空间）---
# health_monitor：检查 required_nodes（默认含 heartbeat）→ 通过 set_heartbeat_params 推进
#   robot_status（localizing / localization_lost / ready / shutdown 等）；订阅 ~/robot_status 与定位位姿话题。
# task_manager：暴露 ~/set_robot_task，将 task_status 写入 heartbeat。
# 必须在 heartbeat lifecycle active 之后启动，否则 set_heartbeat_params 与 robot_status 可能不可用。
ros2 launch manager manager.launch.py \
  "namespace:=${RID}" \
  "localization_pose_topic:=${MANAGER_POSE_TOPIC}" &
log "started manager.launch.py (health_monitor+task_manager, pid $!) namespace=${RID} pose_topic=${MANAGER_POSE_TOPIC}"
sleep "${MANAGER_WAIT}"

# --- 3) 定位 + 建图（同时启动，按 lifecycle 切换 active/inactive）---
ros2 launch slam_bringup localization.launch.py \
  "robot_name:=${RID}" \
  "namespace:=${RID}" &
log "started localization.launch.py (pid $!)"

ros2 launch slam_bringup mapping.launch.py \
  "robot_name:=${RID}" \
  "namespace:=${RID}" &
log "started mapping.launch.py (pid $!)"

SLAM_LOCALIZATION="/${RID}/slam_bringup/localization"
SLAM_MAPPING="/${RID}/slam_bringup/mapping"

wait_lifecycle_node() {
  local node="$1"
  local timeout_s="$2"
  local ready=0
  local i
  for i in $(seq 1 "${timeout_s}"); do
    if ros2 lifecycle get "${node}" >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "${ready}" == "1" ]]; then
    return 0
  fi
  return 1
}

lifecycle_set_best_effort() {
  local node="$1"
  local transition="$2"
  if ros2 lifecycle set "${node}" "${transition}" >/dev/null 2>&1; then
    log "slam lifecycle ${transition} requested: ${node}"
  else
    log "slam lifecycle ${transition} skipped/failed (state may already satisfy target): ${node}"
  fi
}

lifecycle_get_state() {
  local node="$1"
  local out
  out="$(ros2 lifecycle get "${node}" 2>/dev/null || true)"
  if [[ "${out}" == *"inactive"* ]]; then
    echo "inactive"
  elif [[ "${out}" == *"active"* ]]; then
    echo "active"
  elif [[ "${out}" == *"unconfigured"* ]]; then
    echo "unconfigured"
  elif [[ "${out}" == *"finalized"* ]]; then
    echo "finalized"
  else
    echo "unknown"
  fi
}

ensure_lifecycle_inactive() {
  local node="$1"
  local state
  state="$(lifecycle_get_state "${node}")"
  case "${state}" in
    unconfigured)
      lifecycle_set_best_effort "${node}" configure
      ;;
    inactive)
      ;;
    active)
      lifecycle_set_best_effort "${node}" deactivate
      ;;
    *)
      # Best effort fallback if state parsing failed.
      lifecycle_set_best_effort "${node}" configure
      lifecycle_set_best_effort "${node}" deactivate
      ;;
  esac
}

ensure_lifecycle_active() {
  local node="$1"
  local state
  state="$(lifecycle_get_state "${node}")"
  case "${state}" in
    active)
      ;;
    inactive)
      lifecycle_set_best_effort "${node}" activate
      ;;
    unconfigured)
      lifecycle_set_best_effort "${node}" configure
      lifecycle_set_best_effort "${node}" activate
      ;;
    *)
      # Best effort fallback if state parsing failed.
      lifecycle_set_best_effort "${node}" configure
      lifecycle_set_best_effort "${node}" activate
      ;;
  esac
}

for slam_node in "${SLAM_LOCALIZATION}" "${SLAM_MAPPING}"; do
  if wait_lifecycle_node "${slam_node}" "${SLAM_WAIT}"; then
    ensure_lifecycle_inactive "${slam_node}"
  else
    log "WARN: slam lifecycle service not visible within ${SLAM_WAIT}s: ${slam_node}"
  fi
done

if [[ "${AUTO_MAPPING}" == "1" ]]; then
  # mapping mode: first force localization inactive, then bring mapping active.
  ensure_lifecycle_inactive "${SLAM_LOCALIZATION}"
  ensure_lifecycle_active "${SLAM_MAPPING}"
  log "slam mode=mapping (localization inactive, mapping active)"
else
  # normal mode: first force mapping inactive, then bring localization active.
  ensure_lifecycle_inactive "${SLAM_MAPPING}"
  ensure_lifecycle_active "${SLAM_LOCALIZATION}"
  log "slam mode=normal (localization active, mapping inactive)"
fi

# --- 5) 导航（默认 active）---
ros2 launch nav_bringup stack.launch.py \
  "robot_name:=${RID}" \
  "grid_mode:=${NAV_GRID_MODE}" \
  "autostart:=true" &
log "started nav stack.launch.py (pid $!)"

NM="/${RID}/lifecycle_manager_navigation"
for _ in $(seq 1 "${NAV_WAIT}"); do
  if ros2 node list 2>/dev/null | grep -q "^${NM}$"; then
    log "navigation lifecycle manager node visible at ${NM} (autostart=true)"
    break
  fi
  sleep 1
done

log "all stack processes started; waiting on background jobs (simulate is long-lived)"
wait
