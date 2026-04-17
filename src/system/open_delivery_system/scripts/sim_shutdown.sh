#!/usr/bin/env bash
# Web「仿真离线」：结束该 robot 命名空间下由 sim_bringup 拉起的栈（不含 Gazebo 本体），
# 再根据是否仍存在 /robot*/robot_status 话题决定是否关闭 Gazebo。
#
# Usage: OPEN_DELIVERY_ROOT=/path/to/openDelivery sim_shutdown.sh <robot_id>

set -eo pipefail

log() {
  echo "[sim_shutdown $(date -Iseconds)] $*" >&2
}

log_match_count() {
  local pattern="$1"
  local label="$2"
  local count
  count="$(pgrep -f "${pattern}" 2>/dev/null | wc -l || true)"
  log "${label}: match_count=${count}"
}

RID="${1:?robot id required (e.g. robot2)}"

ROOT="${OPEN_DELIVERY_ROOT:-}"
if [[ -z "${ROOT}" ]]; then
  _here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  ROOT="$(cd "${_here}/../../../../" && pwd)"
fi

ROS_DISTRO="${ROS_DISTRO:-foxy}"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${ROOT}/install/setup.bash"
fi
cd "${ROOT}" 2>/dev/null || true

log "robot_id=${RID} OPEN_DELIVERY_ROOT=${ROOT}"

# 与 sim_bringup 栈顺序相反；仅杀本机命名空间 /${RID}/ 相关 launch（Gazebo 进程名单独处理）
kill_stack_for_robot() {
  local r="$1"
  log "pkill ros2 stacks for namespace/robot_name=${r}"
  log_match_count "ros2 launch nav_bringup stack.launch.py.*robot_name:=${r}" "before nav_bringup"
  pkill -f "ros2 launch nav_bringup stack.launch.py.*robot_name:=${r}" 2>/dev/null || true
  log_match_count "ros2 launch nav_bringup stack.launch.py.*robot_name:=${r}" "after nav_bringup"
  log_match_count "ros2 launch slam_bringup mapping.launch.py.*robot_name:=${r}" "before slam_mapping"
  pkill -f "ros2 launch slam_bringup mapping.launch.py.*robot_name:=${r}" 2>/dev/null || true
  log_match_count "ros2 launch slam_bringup mapping.launch.py.*robot_name:=${r}" "after slam_mapping"
  log_match_count "ros2 launch slam_bringup localization.launch.py.*robot_name:=${r}" "before slam_localization"
  pkill -f "ros2 launch slam_bringup localization.launch.py.*robot_name:=${r}" 2>/dev/null || true
  log_match_count "ros2 launch slam_bringup localization.launch.py.*robot_name:=${r}" "after slam_localization"
  log_match_count "ros2 launch simulate simulate.launch.py.*robot_name:=${r}" "before simulate"
  pkill -f "ros2 launch simulate simulate.launch.py.*robot_name:=${r}" 2>/dev/null || true
  log_match_count "ros2 launch simulate simulate.launch.py.*robot_name:=${r}" "after simulate"
  # 托管入口：web 侧用 bash sim_bringup.sh <id>
  log_match_count "sim_bringup.sh.*${r}" "before sim_bringup_entry"
  pkill -f "sim_bringup.sh.*${r}" 2>/dev/null || true
  log_match_count "sim_bringup.sh.*${r}" "after sim_bringup_entry"

  # Fallback: some ROS2 nodes may survive with direct node executables.
  # Most namespaced nodes carry "__ns:=/<robot>" in argv.
  log_match_count "__ns:=/${r}( |$)" "before namespace_fallback"
  pkill -f "__ns:=/${r}( |$)" 2>/dev/null || true
  log_match_count "__ns:=/${r}( |$)" "after namespace_fallback"

  # Heartbeat always kill at the very end.
  log_match_count "ros2 launch heartbeat heartbeat.launch.py.*namespace:=${r}" "before heartbeat(last)"
  pkill -f "ros2 launch heartbeat heartbeat.launch.py.*namespace:=${r}" 2>/dev/null || true
  log_match_count "ros2 launch heartbeat heartbeat.launch.py.*namespace:=${r}" "after heartbeat(last)"
}

kill_stack_for_robot "${RID}"

SLEEP_SEC="${SIM_SHUTDOWN_TOPIC_WAIT_SEC:-3}"
log "sleep ${SLEEP_SEC}s before checking remaining /robot*/robot_status"
sleep "${SLEEP_SEC}"

# 打印该机器人命名空间是否仍有残留节点，便于定位离线失败原因
if command -v ros2 >/dev/null 2>&1; then
  rem_nodes="$(
    timeout 6 ros2 node list 2>/dev/null | grep -E "^/${RID}/" || true
  )"
  if [[ -n "${rem_nodes// }" ]]; then
    log "WARN: remaining nodes in /${RID}/ after shutdown attempt:"
    while IFS= read -r line; do
      [[ -n "${line}" ]] && log "  ${line}"
    done <<<"${rem_nodes}"
  else
    log "no remaining nodes under /${RID}/"
  fi
fi

# 若已无任意 /<id>/robot_status，则关闭 Gazebo（多机仿真时仅最后一台离线才关）
remaining=""
if command -v ros2 >/dev/null 2>&1; then
  log "ros2 found, checking remaining robot_status topics"
  remaining="$(
    timeout 6 ros2 topic list 2>/dev/null | grep -E '^/[a-zA-Z0-9_][a-zA-Z0-9_]*/robot_status$' || true
  )"
else
  log "ros2 not found in PATH, skip topic check and keep Gazebo decision by empty result"
fi

if [[ -z "${remaining// }" ]]; then
  log "no /robot*/robot_status topics left -> stopping gzserver (and gzclient if present)"
  log_match_count "gzserver" "before gzserver"
  log_match_count "gzclient" "before gzclient"
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
  log_match_count "gzserver" "after gzserver"
  log_match_count "gzclient" "after gzclient"
else
  log "still have robot_status topic(s), keep Gazebo running:"
  while IFS= read -r line; do
    [[ -n "${line}" ]] && log "  ${line}"
  done <<<"${remaining}"
fi

log "done"
