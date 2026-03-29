#!/usr/bin/env bash
set -euo pipefail
# Usage: ROBOT_NAME=robot2 start_open_delivery.sh
#        start_open_delivery.sh robot3
PKG="open_delivery_system"
if ! ros2 pkg prefix "${PKG}" &>/dev/null; then
  echo "open_delivery_system not found. Source your workspace: source install/setup.bash" >&2
  exit 1
fi
ROBOT_NAME="${1:-${ROBOT_NAME:-robot2}}"
exec ros2 launch "${PKG}" "startup.launch.py" "robot_name:=${ROBOT_NAME}"
