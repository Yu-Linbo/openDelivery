#!/usr/bin/env bash
#
# =============================================================================
# Web「仿真上线」唯一入口（本仓库 sim 栈）
# =============================================================================
# 在本脚本内**顺序后台启动**同一命名空间 ${RID} 下的：
#   1) simulate（Gazebo + 机器人）
#   2) heartbeat（RobotStatus 发布；Lifecycle 需 configure→activate）
#   3) manager（health_monitor + task_manager + lifecycle_manager）
#   4) lifecycle_manager 按 initial_slam_mode 切换 slam（mapping|localize）
#   5) nav_bringup stack
#
# **仿真下线**不再调用 sim_shutdown.sh：由上层将 ``RobotStatus.robot_status`` 置为 **shutdown**，
# 典型路径为 Web「仿真离线」→ ``robot_lifecycle.shutdown_selected_robot`` →
# ``ros2 service call /<id>/set_heartbeat_params``；
# simulate / nav / slam 等应订阅 ``/<id>/robot_status`` 或监听该状态并自行 pkill 或 lifecycle 收尾
#（本脚本不负责杀进程）。
#
# 用法:
#   OPEN_DELIVERY_ROOT=/path/to/openDelivery SIM_MODE=sim sim_bringup.sh <robot_id>
# 可选环境变量:
#   SIM_BRINGUP_VERBOSE=1          打开 bash -x
#   SIM_BRINGUP_MANAGER_ONLY=1     仅起 simulate + heartbeat + manager（health_monitor/task_manager），跳过 slam/nav（测 manager 用）
#   SIM_BRINGUP_MANAGER_WAIT=2   heartbeat 就绪后等待秒数再拉 manager（默认 2）
#   SIM_BRINGUP_MANAGER_POSE_TOPIC=amcl_pose   传给 health_monitor；置空则关闭位姿判定 ready
#   SIM_BRINGUP_MAX_BAG_BYTES=5242880          rosbag 单包上限（默认 5MB，测试轮转用）
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

# Foxy FastRTPS may ignore FASTDDS_BUILTIN_TRANSPORTS and attempt stale SHM
# segments. Apply the XML profile before any ROS process is created so manager,
# map_server, SLAM, Nav2 and Gazebo all use the same UDP-only transport.
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
: "${FASTRTPS_DEFAULT_PROFILES_FILE:=${ROOT}/backend/fastdds_udp_only.xml}"
export FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE

log "robot_id=${RID} SIM_MODE=${SIM_MODE} OPEN_DELIVERY_ROOT=${ROOT}"
log "FastRTPS profile=${FASTRTPS_DEFAULT_PROFILES_FILE}"

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

STORE="${OPEN_DELIVERY_STATUS_DB_PATH:-${ROOT}/backend/data/robot_status_last.json}"
AUTO_MAPPING=0
PERSISTED_MAP=""
if [[ -f "${STORE}" ]]; then
  # Prints: <auto_mapping 0|1> <current_map>
  _persist="$(python3 -c "
import json, sys
rid, path = sys.argv[1], sys.argv[2]
try:
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    e = d.get(rid) or {}
    s = str(e.get('task_status') or '').strip().lower()
    cm = str(e.get('current_map') or '').strip()
    print(('1' if s == 'mapping' else '0') + ' ' + cm)
except Exception:
    print('0 ')
" "${RID}" "${STORE}")"
  AUTO_MAPPING="${_persist%% *}"
  PERSISTED_MAP="${_persist#* }"
  if [[ "${PERSISTED_MAP}" == "${AUTO_MAPPING}" ]]; then
    PERSISTED_MAP=""
  fi
fi
log "AUTO_MAPPING=${AUTO_MAPPING} (from persisted robot_status in ${STORE})"

resolve_floor_map_yaml() {
  # Args: preferred floor name. Echo absolute yaml path or empty.
  local prefer="$1"
  local map_root="${ROOT}/map"
  local name yaml
  if [[ -n "${prefer}" && "${prefer}" != *_mapping ]]; then
    yaml="${map_root}/${prefer}/${prefer}.yaml"
    if [[ -f "${yaml}" ]]; then
      echo "${yaml}"
      return 0
    fi
  fi
  for name in test_101 test_102 test_103 test_104 nh_102 nh_103 nh_113 nh_114 nh_117; do
    yaml="${map_root}/${name}/${name}.yaml"
    if [[ -f "${yaml}" ]]; then
      echo "${yaml}"
      return 0
    fi
  done
  if [[ -d "${map_root}" ]]; then
    for yaml in "${map_root}"/*/*.yaml; do
      [[ -f "${yaml}" ]] || continue
      name="$(basename "$(dirname "${yaml}")")"
      if [[ "$(basename "${yaml}")" == "${name}.yaml" && "${name}" != *_mapping ]]; then
        echo "${yaml}"
        return 0
      fi
    done
  fi
  echo ""
}

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

MAX_BAG_BYTES="${SIM_BRINGUP_MAX_BAG_BYTES:-5242880}"
log "MAX_BAG_BYTES=${MAX_BAG_BYTES}"

HB_CURRENT_MAP="${RID}_mapping"
MAP_FILE=""
if [[ "${AUTO_MAPPING}" == "1" ]]; then
  HB_CURRENT_MAP="${RID}_mapping"
  # Optional: keep last floor on /<robot>/map via map_server; mapping stream stays on /mapping.
  MAP_FILE="$(resolve_floor_map_yaml "${PERSISTED_MAP}")"
else
  MAP_FILE="$(resolve_floor_map_yaml "${PERSISTED_MAP}")"
  if [[ -n "${MAP_FILE}" ]]; then
    HB_CURRENT_MAP="$(basename "$(dirname "${MAP_FILE}")")"
  else
    log "WARN: no floor map yaml under ${ROOT}/map; localize needs map_file for map_server"
    HB_CURRENT_MAP="${PERSISTED_MAP:-${RID}_mapping}"
  fi
fi
log "heartbeat current_map=${HB_CURRENT_MAP} map_file=${MAP_FILE:-<none>}"

# --- 0) 基础栈：日志/rosbag + heartbeat（统一由 params/startup.launch.py 维护）---
ros2 launch system startup.launch.py \
  "robot_name:=${RID}" \
  "current_map:=${HB_CURRENT_MAP}" \
  "robot_status:=initializing" \
  "sim_mode:=${SIM_MODE}" \
  "mapping_mode:=false" \
  "publish_rate:=2.0" \
  "log_root:=${ROOT}/log_bag" \
  "max_bag_bytes:=${MAX_BAG_BYTES}" \
  "enable_fake_pub:=false" &
log "started system startup.launch.py (log_bag+heartbeat, pid $!)"

# --- 1) Gazebo + 机器人 ---
ros2 launch simulate simulate.launch.py \
  "robot_name:=${RID}" \
  "namespace:=${RID}" \
  "start_gazebo:=${START_GZ}" \
  "spawn_robot:=true" \
  "use_sim_time:=true" &
log "started simulate.launch.py (pid $!)"
sleep "${SIM_WAIT}"

# --- 2) 心跳 lifecycle 激活（节点由 startup.launch.py 启动）---
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

# --- 2b) manager：health_monitor + task_manager + lifecycle_manager ---
SLAM_INITIAL_MODE="inactive"
MANAGER_EXTRA=()
if [[ -n "${MAP_FILE}" ]]; then
  MANAGER_EXTRA+=("map_file:=${MAP_FILE}")
fi
if [[ "${AUTO_MAPPING}" == "1" ]]; then
  SLAM_INITIAL_MODE="mapping"
else
  SLAM_INITIAL_MODE="localize"
fi
ros2 launch manager manager.launch.py \
  "namespace:=${RID}" \
  "localization_pose_topic:=${MANAGER_POSE_TOPIC}" \
  "use_sim_time:=true" \
  "initial_slam_mode:=${SLAM_INITIAL_MODE}" \
  "${MANAGER_EXTRA[@]}" &
log "started manager.launch.py (health_monitor+task_manager+lifecycle_manager, pid $!) namespace=${RID} slam=${SLAM_INITIAL_MODE}"
sleep "${MANAGER_WAIT}"

if [[ "${SIM_BRINGUP_MANAGER_ONLY:-0}" == "1" ]]; then
  log "SIM_BRINGUP_MANAGER_ONLY=1: skip nav; background jobs = simulate + heartbeat + manager"
  wait
  exit 0
fi

# --- 3) SLAM mode is owned by lifecycle_manager (initial_slam_mode above) ---

# --- 4) 导航（默认 active）---
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
