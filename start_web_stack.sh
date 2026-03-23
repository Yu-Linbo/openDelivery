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

# 默认 ROBOT_POSE_MODE=ros2_tf（真 TF）；无 ROS 时位姿列表为空。仅演示轨迹请: export ROBOT_POSE_MODE=mock
: "${ROBOT_POSE_MODE:=ros2_tf}"
export ROBOT_POSE_MODE

echo "[open-delivery] starting backend on ${BACKEND_HOST}:${BACKEND_PORT} (ROBOT_POSE_MODE=${ROBOT_POSE_MODE})"
MAP_API_PORT="${BACKEND_PORT}" MAP_API_HOST="${BACKEND_HOST}" python3 "${BACKEND_SCRIPT}" &
BACKEND_PID=$!

echo "[open-delivery] starting frontend on 0.0.0.0:${FRONTEND_PORT}"
python3 -m http.server "${FRONTEND_PORT}" --directory "${WEB_DIR}" &
FRONTEND_PID=$!

cleanup() {
  echo
  echo "[open-delivery] stopping services..."
  kill "${BACKEND_PID}" "${FRONTEND_PID}" >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

echo "[open-delivery] ready:"
echo "  frontend: http://localhost:${FRONTEND_PORT}"
echo "  backend : http://localhost:${BACKEND_PORT}"
echo "Press Ctrl+C to stop."

wait -n "${BACKEND_PID}" "${FRONTEND_PID}"
