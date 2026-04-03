#!/usr/bin/env bash
# One-shot: heartbeat + slam_bringup + nav_bringup (grid_mode localize→/<robot>/map, mapping→/<robot>/mapping; no /map relay).
# Modes are mutually exclusive: normal = localization, mapping = mapping (not both).
#
# Does NOT start Gazebo/robot/sensors — run simulate (or real stack) separately so /clock, /odom, /scan_2d exist.
#
# Usage:
#   ./start_heartbeat_slam_nav.sh <robot_name> mapping
#   ./start_heartbeat_slam_nav.sh <robot_name> normal [/absolute/path/to/map.yaml]
#
# 建图推荐：先起仿真（与 robot 同名 namespace），再
#   ./start_heartbeat_slam_nav.sh robot2 mapping
#
# Debug: ros2 topic echo /robot2/robot_status
#   default echo uses best_effort; heartbeat publishes reliable → use:
#   ros2 topic echo /robot2/robot_status --qos-reliability reliable
#
# Env (optional):
#   USE_SIM_TIME=true|false   (default: true)
#   MAP_FILE                  used for normal mode if map yaml not passed as 3rd arg
#   CURRENT_MAP               heartbeat current_map (default: nh_102)
#   OPEN_DELIVERY_ROOT        optional repo root for other tools (default: this repo root)

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
# ROS/setup.bash may reference unset optional env vars (e.g. AMENT_TRACE_SETUP_FILES).
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

# heartbeat.launch.py has no use_sim_time; slam + navigation do.
SIM_ARG="use_sim_time:=$USE_SIM_TIME"

echo "[start_heartbeat_slam_nav] robot=${ROBOT} mode=${MODE} USE_SIM_TIME=${USE_SIM_TIME} OPEN_DELIVERY_ROOT=${OPEN_DELIVERY_ROOT}"

if [[ "$MODE" == "mapping" ]]; then
  ros2 launch heartbeat heartbeat.launch.py \
    "namespace:=${ROBOT}" \
    "mapping_mode:=true" \
    "current_map:=${CURRENT_MAP}" &
  PIDS+=("$!")

  ros2 launch slam_bringup mapping.launch.py \
    "robot_name:=${ROBOT}" \
    "namespace:=${ROBOT}" \
    "${SIM_ARG}" &
  PIDS+=("$!")

  ros2 launch nav_bringup stack.launch.py \
    "robot_name:=${ROBOT}" \
    "grid_mode:=mapping" \
    "${SIM_ARG}" &
  PIDS+=("$!")
else
  ros2 launch heartbeat heartbeat.launch.py \
    "namespace:=${ROBOT}" \
    "mapping_mode:=false" \
    "current_map:=${CURRENT_MAP}" &
  PIDS+=("$!")

  ros2 launch slam_bringup localization.launch.py \
    "robot_name:=${ROBOT}" \
    "namespace:=${ROBOT}" \
    "map_file:=${MAP_YAML}" \
    "${SIM_ARG}" &
  PIDS+=("$!")

  ros2 launch nav_bringup stack.launch.py \
    "robot_name:=${ROBOT}" \
    "grid_mode:=localize" \
    "${SIM_ARG}" &
  PIDS+=("$!")
fi

echo "[start_heartbeat_slam_nav] started PIDs: ${PIDS[*]} — Ctrl+C to stop all"
wait
