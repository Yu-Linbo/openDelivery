#!/usr/bin/env bash
# One-shot: heartbeat + manager (stack_lifecycle_manager) + nav_bringup.
# Modes are mutually exclusive: normal = localization, mapping = mapping (not both).
#
# Does NOT start Gazebo/robot/sensors — run simulate (or real stack) separately so /clock, /odom, /scan_2d exist.
#
# Usage:
#   ./start_heartbeat_slam_nav.sh <robot_name> mapping
#   ./start_heartbeat_slam_nav.sh <robot_name> normal [/absolute/path/to/map.yaml]
#
# Debug: ros2 topic echo /robot2/stack_lifecycle
#   ros2 service call /robot2/slam/set_stack_lifecycle_transition \
#     custom_msgs_srvs/srv/SetStackLifecycleTransition "{node_name: slam, transition: mapping}"

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OPEN_DELIVERY_ROOT="${OPEN_DELIVERY_ROOT:-$ROOT_DIR}"

usage() {
  echo "Usage: $0 <robot_name> <normal|mapping> [map_yaml_for_normal]" >&2
  echo "  normal   : localization + heartbeat + nav_bringup (navigate_to_pose, etc.)" >&2
  echo "  mapping  : mapping + heartbeat + nav_bringup" >&2
  echo "  For normal, pass map yaml as 3rd arg or set MAP_FILE" >&2
  exit 1
}

if [[ $# -lt 2 ]]; then
  usage
fi

ROBOT="${1:?robot_name}"
MODE="${2,,}"

if [[ "$MODE" != "normal" && "$MODE" != "mapping" ]]; then
  echo "error: mode must be 'normal' or 'mapping', got '$MODE'" >&2
  usage
fi

MAP_YAML="${3:-${MAP_FILE:-}}"

if [[ "$MODE" == "normal" ]]; then
  if [[ -z "$MAP_YAML" ]]; then
    echo "error: normal mode requires a map yaml (3rd arg or MAP_FILE env)" >&2
    exit 1
  fi
  if [[ ! -f "$MAP_YAML" ]]; then
    echo "error: map file not found: $MAP_YAML" >&2
    exit 1
  fi
  MAP_YAML="$(readlink -f "$MAP_YAML")"
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO:-foxy}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "error: missing $ROS_SETUP (set ROS_DISTRO?)" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"

INSTALL_SETUP="${ROOT_DIR}/install/setup.bash"
if [[ ! -f "$INSTALL_SETUP" ]]; then
  echo "error: ${INSTALL_SETUP} not found; run: cd ${ROOT_DIR} && colcon build" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$INSTALL_SETUP"
set -u

USE_SIM_TIME="${USE_SIM_TIME:-true}"
CURRENT_MAP="${CURRENT_MAP:-nh_102}"
STARTUP_TF_WAIT_SEC="${STARTUP_TF_WAIT_SEC:-15}"
STARTUP_REQUIRE_TF="${STARTUP_REQUIRE_TF:-true}"

PIDS=()
cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

SIM_ARG="use_sim_time:=$USE_SIM_TIME"

wait_for_robot_tf_ready() {
  local robot="$1"
  local wait_sec="$2"
  local parent_frame="${robot}/odom"
  local child_frame="${robot}/base_footprint"
  local deadline=$((SECONDS + wait_sec))
  echo "[start_heartbeat_slam_nav] waiting TF ${parent_frame} -> ${child_frame} (<= ${wait_sec}s)"

  while (( SECONDS < deadline )); do
    if python3 - "$parent_frame" "$child_frame" <<'PY' >/dev/null 2>&1
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

parent = sys.argv[1]
child = sys.argv[2]
rclpy.init(args=None)
node = rclpy.create_node("heartbeat_tf_wait")
buf = Buffer(cache_time=Duration(seconds=2.0))
_listener = TransformListener(buf, node, spin_thread=False)
ok = False
deadline_ns = node.get_clock().now().nanoseconds + int(1.2e9)
while rclpy.ok() and node.get_clock().now().nanoseconds < deadline_ns:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.05))
        ok = True
        break
    except Exception:
        pass
node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
    then
      echo "[start_heartbeat_slam_nav] TF ready: ${parent_frame} -> ${child_frame}"
      return 0
    fi
    sleep 0.4
  done

  echo "[start_heartbeat_slam_nav] warn: TF ${parent_frame} -> ${child_frame} not ready within ${wait_sec}s; continue startup"
  return 1
}

stack_transition() {
  local node_name="$1"
  local transition="$2"
  ros2 service call "//slam/set_stack_lifecycle_transition" \
    custom_msgs_srvs/srv/SetStackLifecycleTransition \
    "{node_name: '${node_name}', transition: '${transition}'}" || true
}

echo "[start_heartbeat_slam_nav] robot=${ROBOT} mode=${MODE} USE_SIM_TIME=${USE_SIM_TIME}"

if ! wait_for_robot_tf_ready "$ROBOT" "$STARTUP_TF_WAIT_SEC"; then
  if [[ "${STARTUP_REQUIRE_TF,,}" == "true" || "${STARTUP_REQUIRE_TF}" == "1" || "${STARTUP_REQUIRE_TF,,}" == "yes" ]]; then
    echo "[start_heartbeat_slam_nav] error: required TF not ready for ${ROBOT}" >&2
    exit 1
  fi
fi

if [[ "$MODE" == "mapping" ]]; then
  SLAM_MODE="mapping"
  MAPPING_MODE="true"
  GRID_MODE="mapping"
  MAP_FILE_ARG=""
else
  SLAM_MODE="localize"
  MAPPING_MODE="false"
  GRID_MODE="localize"
  MAP_FILE_ARG="map_file:=${MAP_YAML}"
fi

ros2 launch heartbeat heartbeat.launch.py \
  "namespace:=${ROBOT}" \
  "mapping_mode:=${MAPPING_MODE}" \
  "current_map:=${CURRENT_MAP}" &
PIDS+=("$!")

ros2 launch manager manager.launch.py \
  "namespace:=${ROBOT}" \
  "semantic_map_root:=${ROOT_DIR}/map" \
  "${SIM_ARG}" \
  "initial_slam_mode:=${SLAM_MODE}" \
  "${MAP_FILE_ARG}" &
PIDS+=("$!")

sleep 2
stack_transition heartbeat configure
stack_transition heartbeat activate

ros2 launch nav_bringup stack.launch.py \
  "robot_name:=${ROBOT}" \
  "grid_mode:=${GRID_MODE}" \
  "${SIM_ARG}" &
PIDS+=("$!")

echo "[start_heartbeat_slam_nav] started PIDs: ${PIDS[*]} — Ctrl+C to stop all"
wait
