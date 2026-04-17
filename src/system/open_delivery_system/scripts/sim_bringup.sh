#!/usr/bin/env bash
# Web「仿真上线」唯一入口：在本脚本内顺序拉起 Gazebo/仿真、心跳、定位、（可选）建图、导航。
# 对应下线见 sim_shutdown.sh（杀 /robot*/ 栈并按 robot_status 话题决定是否关 Gazebo）。
# 用法: OPEN_DELIVERY_ROOT=/path/to/openDelivery SIM_MODE=sim sim_bringup.sh <robot_id>
# 环境: ROS_DISTRO（默认 foxy）、SIM_BRINGUP_VERBOSE=1 打开 set -x
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
    s = str(e.get('robot_status', '')).strip().lower()
    print(1 if s == 'mapping' else 0)
except Exception:
    print(0)
" "${RID}" "${STORE}")"
fi
log "AUTO_MAPPING=${AUTO_MAPPING} (from persisted robot_status in ${STORE})"

SIM_WAIT="${SIM_BRINGUP_SIM_WAIT:-6}"
HB_WAIT="${SIM_BRINGUP_HEARTBEAT_WAIT_SEC:-20}"
NAV_WAIT="${SIM_BRINGUP_NAV_WAIT_SEC:-120}"

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

# # --- 3) 定位 ---
# ros2 launch slam_bringup localization.launch.py \
#   "robot_name:=${RID}" \
#   "namespace:=${RID}" &
# log "started localization.launch.py (pid $!)"

# --- 4) 建图（仅历史 robot_status 为 mapping）---
if [[ "${AUTO_MAPPING}" == "1" ]]; then
  ros2 launch slam_bringup mapping.launch.py \
    "robot_name:=${RID}" \
    "namespace:=${RID}" &
  log "started mapping.launch.py (pid $!)"
else
  log "skip mapping.launch.py (persisted status was not mapping)"
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
