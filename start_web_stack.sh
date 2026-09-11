#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="${ROOT_DIR}/web"
BACKEND_SCRIPT="${ROOT_DIR}/backend/server.py"

FRONTEND_PORT="${FRONTEND_PORT:-8000}"
BACKEND_PORT="${BACKEND_PORT:-8001}"
BACKEND_HOST="${BACKEND_HOST:-0.0.0.0}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 未安装，无法启动。"
  exit 1
fi
if ! python3 -c "from PIL import Image" >/dev/null 2>&1; then
  echo "python3-pil / Pillow 未安装，地图编辑 API 无法启动。"
  exit 1
fi

# Keep generated messages and every compiled consumer on the same ABI. Merely
# importing the Python message does not prove that downstream C++ nodes were
# rebuilt after a .msg/.srv change.
source_setup_safely() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    # setup files may reference optional variables while this script uses set -u.
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
  fi
}

ROS_DISTRO_NAME="${ROS_DISTRO:-foxy}"
# Set this before ROS setup, which otherwise supplies a default of 0. Local
# simulation must not depend on discovery through VPN/proxy network adapters.
# Set ROS_LOCALHOST_ONLY=0 explicitly when connecting physical/remote robots.
: "${ROS_LOCALHOST_ONLY:=1}"
export ROS_LOCALHOST_ONLY
source_setup_safely "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

CUSTOM_MSG_SOURCE="${ROOT_DIR}/src/common/custom_msgs_srvs"
CUSTOM_MSG_STAMP="${ROOT_DIR}/build/.open_delivery_custom_msgs_abi.sha256"
CUSTOM_MSG_HASH="$(
  find "${CUSTOM_MSG_SOURCE}" -maxdepth 2 -type f \
    \( -name '*.msg' -o -name '*.srv' -o -name 'CMakeLists.txt' -o -name 'package.xml' \) \
    -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
)"
NEEDS_INTERFACE_BUILD=0
if [ ! -f "${ROOT_DIR}/install/setup.bash" ] \
  || [ ! -f "${CUSTOM_MSG_STAMP}" ] \
  || [ "$(tr -d '[:space:]' < "${CUSTOM_MSG_STAMP}" 2>/dev/null || true)" != "${CUSTOM_MSG_HASH}" ]; then
  NEEDS_INTERFACE_BUILD=1
else
  source_setup_safely "${ROOT_DIR}/install/setup.bash"
  if ! python3 -c "from custom_msgs_srvs.msg import RobotStatus" >/dev/null 2>&1; then
    NEEDS_INTERFACE_BUILD=1
  fi
fi

if [ "${NEEDS_INTERFACE_BUILD}" -eq 1 ]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "[open-delivery] colcon not found; cannot rebuild custom message consumers."
    exit 1
  fi
  echo "[open-delivery] custom message ABI changed; rebuilding all dependent packages ..."
  (
    cd "${ROOT_DIR}"
    colcon build \
      --packages-above-and-dependencies custom_msgs_srvs \
      --symlink-install \
      --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      --event-handlers console_direct+
  )
  mkdir -p "${ROOT_DIR}/build"
  printf '%s\n' "${CUSTOM_MSG_HASH}" > "${CUSTOM_MSG_STAMP}"
  source_setup_safely "${ROOT_DIR}/install/setup.bash"
fi

if ! python3 -c "from custom_msgs_srvs.msg import RobotStatus" >/dev/null 2>&1; then
  echo "[open-delivery] custom_msgs_srvs is still unavailable after build."
  exit 1
fi

# A systemd restart represents a restart of the whole local OpenDelivery
# runtime.  Clean up stale Web, ROS 2 and Gazebo processes before binding ports;
# the same cleanup is also run from the TERM/EXIT trap below.
is_current_or_ancestor_pid() {
  local candidate="$1"
  local cursor="$$"
  while [[ "${cursor}" =~ ^[0-9]+$ ]] && [[ "${cursor}" -gt 1 ]]; do
    if [[ "${candidate}" -eq "${cursor}" ]]; then
      return 0
    fi
    cursor="$(awk '/^PPid:/ { print $2 }' "/proc/${cursor}/status" 2>/dev/null || true)"
  done
  return 1
}

collect_old_web_pids() {
  local proc_dir pid cmd cwd
  for proc_dir in /proc/[0-9]*; do
    pid="${proc_dir##*/}"
    is_current_or_ancestor_pid "${pid}" && continue
    cmd="$(tr '\0' ' ' <"${proc_dir}/cmdline" 2>/dev/null || true)"
    cwd="$(readlink -f "${proc_dir}/cwd" 2>/dev/null || true)"
    if [[ "${cmd}" == *"${ROOT_DIR}/start_web_stack.sh"* ]] \
      || [[ "${cmd}" == *"${BACKEND_SCRIPT}"* ]] \
      || { [[ "${cwd}" == "${WEB_DIR}" ]] && [[ "${cmd}" == *"python3"* ]]; }; then
      echo "${pid}"
    fi
  done
}

stop_old_web_stack() {
  local pid alive
  local -a old_pids=()
  while IFS= read -r pid; do
    [[ -n "${pid}" ]] && old_pids+=("${pid}")
  done < <(collect_old_web_pids)
  if [[ "${#old_pids[@]}" -eq 0 ]]; then
    return
  fi

  echo "[open-delivery] stopping stale web stack PIDs: ${old_pids[*]}"
  kill -TERM "${old_pids[@]}" >/dev/null 2>&1 || true
  for _ in {1..20}; do
    alive=0
    for pid in "${old_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done
    [[ "${alive}" -eq 0 ]] && return
    sleep 0.1
  done
  for pid in "${old_pids[@]}"; do
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  done
}

stop_old_web_stack

collect_ros_gazebo_pids() {
  local proc_dir pid cmd exe base
  for proc_dir in /proc/[0-9]*; do
    pid="${proc_dir##*/}"
    is_current_or_ancestor_pid "${pid}" && continue
    cmd="$(tr '\0' ' ' <"${proc_dir}/cmdline" 2>/dev/null || true)"
    [[ -z "${cmd}" ]] && continue
    exe="$(readlink -f "${proc_dir}/exe" 2>/dev/null || true)"
    base="${exe##*/}"
    if [[ "${base}" == "gzserver" || "${base}" == "gzclient" || "${base}" == "gazebo" ]] \
      || [[ "${base}" == "_ros2_daemon" ]] \
      || [[ "${base}" == "robot_log_recorder" ]] \
      || [[ "${exe}" == /opt/ros/*/lib/* ]] \
      || [[ "${cmd}" =~ (^|[[:space:]/])ros2[[:space:]]+(launch|run|bag)([[:space:]]|$) ]] \
      || { [[ "${cmd}" == *"${ROOT_DIR}/install/"* ]] && [[ "${cmd}" != *"${BACKEND_SCRIPT}"* ]]; }; then
      echo "${pid}"
    fi
  done
}

stop_ros_gazebo_runtime() {
  local pid alive
  local -a runtime_pids=()
  while IFS= read -r pid; do
    [[ -n "${pid}" ]] && runtime_pids+=("${pid}")
  done < <(collect_ros_gazebo_pids)
  if [[ "${#runtime_pids[@]}" -eq 0 ]]; then
    return
  fi

  echo "[open-delivery] stopping ROS/Gazebo runtime PIDs: ${runtime_pids[*]}"
  kill -TERM "${runtime_pids[@]}" >/dev/null 2>&1 || true
  for _ in {1..30}; do
    alive=0
    for pid in "${runtime_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done
    [[ "${alive}" -eq 0 ]] && return
    sleep 0.1
  done
  for pid in "${runtime_pids[@]}"; do
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  done
}

stop_ros_gazebo_runtime

port_is_busy() {
  local port="$1"
  if lsof -iTCP:"${port}" -sTCP:LISTEN -t >/dev/null 2>&1; then
    return 0
  fi
  return 1
}

pick_port() {
  local preferred="$1"
  local selected="${preferred}"
  local tries=0
  while port_is_busy "${selected}" && [[ "${tries}" -lt 20 ]]; do
    selected=$((selected + 1))
    tries=$((tries + 1))
  done
  echo "${selected}"
}

FRONTEND_PORT="$(pick_port "${FRONTEND_PORT}")"
BACKEND_PORT="$(pick_port "${BACKEND_PORT}")"
while [[ "${BACKEND_PORT}" -eq "${FRONTEND_PORT}" ]] || port_is_busy "${BACKEND_PORT}"; do
  BACKEND_PORT=$((BACKEND_PORT + 1))
done

# Keep every process on FastRTPS built-in transports. Foxy's custom UDP-only
# profile can discover topics while all ROS service calls still time out.
: "${FASTDDS_BUILTIN_TRANSPORTS:=DEFAULT}"
export FASTDDS_BUILTIN_TRANSPORTS
# ROS 2 Foxy may ignore FASTDDS_BUILTIN_TRANSPORTS, so retain the compatible
# XML participant profile as the authoritative setting.
: "${FASTRTPS_DEFAULT_PROFILES_FILE:=${ROOT_DIR}/backend/fastdds_udp_only.xml}"
export FASTRTPS_DEFAULT_PROFILES_FILE

# Fast DDS built-in transports are required for reliable Foxy service calls,
# but hard-killed participants can leave SHM port locks behind.  The official
# cleaner removes zombie segments only and preserves active participants.
if command -v fastdds >/dev/null 2>&1; then
  fastdds shm clean >/dev/null 2>&1 || true
fi

# 默认 ROBOT_POSE_MODE=ros2_tf（真 TF）；无 ROS 时位姿列表为空。仅演示轨迹请: export ROBOT_POSE_MODE=mock
: "${ROBOT_POSE_MODE:=ros2_tf}"
export ROBOT_POSE_MODE

STOP_ALL=0
BACKEND_PID=""

backend_supervisor() {
  while [[ "${STOP_ALL}" -eq 0 ]]; do
    echo "[open-delivery] starting backend on ${BACKEND_HOST}:${BACKEND_PORT} (ROBOT_POSE_MODE=${ROBOT_POSE_MODE})"
    MAP_API_PORT="${BACKEND_PORT}" MAP_API_HOST="${BACKEND_HOST}" python3 "${BACKEND_SCRIPT}" &
    BACKEND_PID=$!
    wait "${BACKEND_PID}" || true
    if [[ "${STOP_ALL}" -ne 0 ]]; then
      break
    fi
    echo "[open-delivery] backend exited unexpectedly; respawning in 1s..."
    sleep 1
  done
}

backend_supervisor &
BACKEND_SUP_PID=$!

echo "[open-delivery] starting frontend on 0.0.0.0:${FRONTEND_PORT}"
WEB_DIR="${WEB_DIR}" FRONTEND_PORT="${FRONTEND_PORT}" python3 - <<'PY' &
import http.server
import os


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


os.chdir(os.environ["WEB_DIR"])
port = int(os.environ["FRONTEND_PORT"])
http.server.ThreadingHTTPServer(("0.0.0.0", port), NoCacheHandler).serve_forever()
PY
FRONTEND_PID=$!

cleanup() {
  STOP_ALL=1
  echo
  echo "[open-delivery] stopping services..."
  stop_ros_gazebo_runtime
  pkill -f "${BACKEND_SCRIPT}" >/dev/null 2>&1 || true
  kill "${BACKEND_SUP_PID}" "${FRONTEND_PID}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

echo "[open-delivery] ready:"
echo "  frontend: http://localhost:${FRONTEND_PORT}"
echo "  backend : http://localhost:${BACKEND_PORT}"
echo "Press Ctrl+C to stop."

wait -n "${BACKEND_SUP_PID}" "${FRONTEND_PID}"
