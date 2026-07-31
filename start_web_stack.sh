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

# Ensure custom message package is available for web/ROS bridge.
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  # `install/setup.bash` may reference optional env vars that are not set.
  # Since this script runs with `set -u`, temporarily disable it while sourcing.
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
  if ! python3 -c "from custom_msgs_srvs.msg import RobotStatus" >/dev/null 2>&1; then
    if command -v colcon >/dev/null 2>&1; then
      echo "[open-delivery] building custom_msgs_srvs ..."
      (cd "${ROOT_DIR}" && colcon build --packages-select custom_msgs_srvs --event-handlers console_direct+ || true)
      # shellcheck disable=SC1090
      set +u
      source "${ROOT_DIR}/install/setup.bash"
      set -u
    else
      echo "[open-delivery] colcon not found; custom_msgs_srvs may be unavailable."
    fi
  fi
fi

# Stop stale copies of this project's web stack before choosing ports. The
# frontend below is launched as `python3 -`, so its cwd is the reliable project
# boundary. ROS/Gazebo/robot processes are deliberately not touched here.
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

# Avoid FastRTPS shared-memory errors after hard kills (stale /dev/shm segments).
# UDP-only transport is more reliable for the web backend joining an existing ROS graph.
: "${FASTDDS_BUILTIN_TRANSPORTS:=UDPv4}"
export FASTDDS_BUILTIN_TRANSPORTS
# ROS 2 Foxy uses FastRTPS versions that may ignore FASTDDS_BUILTIN_TRANSPORTS.
# An XML participant profile is the compatible way to disable shared memory.
: "${FASTRTPS_DEFAULT_PROFILES_FILE:=${ROOT_DIR}/backend/fastdds_udp_only.xml}"
export FASTRTPS_DEFAULT_PROFILES_FILE

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
  pkill -f "${BACKEND_SCRIPT}" >/dev/null 2>&1 || true
  kill "${BACKEND_SUP_PID}" "${FRONTEND_PID}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

echo "[open-delivery] ready:"
echo "  frontend: http://localhost:${FRONTEND_PORT}"
echo "  backend : http://localhost:${BACKEND_PORT}"
echo "Press Ctrl+C to stop."

wait -n "${BACKEND_SUP_PID}" "${FRONTEND_PID}"
