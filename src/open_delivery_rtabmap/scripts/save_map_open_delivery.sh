#!/usr/bin/env bash
set -eo pipefail
# 将当前 /map (RTAB-Map 发布的 OccupancyGrid) 存为 Nav2 风格 pgm+yaml 到 openDelivery/map/<name>/
# 用法: save_map_open_delivery.sh <name>
# 例:   save_map_open_delivery.sh floor1

NAME="${1:-}"
if [ -z "${NAME}" ]; then
  echo "用法: $0 <地图名>" >&2
  echo "文件将写入: \${OPENDELIVERY_MAP_DIR}/${NAME:-<name>}/<name>.{pgm,yaml}" >&2
  exit 1
fi

: "${OPENDELIVERY_ROOT:?Set OPENDELIVERY_ROOT to the openDelivery directory.}"

ROS_DISTRO="${ROS_DISTRO:-foxy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${OPENDELIVERY_ROOT}/install/setup.bash"
export OPENDELIVERY_MAP_DIR="${OPENDELIVERY_MAP_DIR:-${OPENDELIVERY_ROOT}/map}"

OUT_DIR="${OPENDELIVERY_MAP_DIR}/${NAME}"
mkdir -p "${OUT_DIR}"
PREFIX="${OUT_DIR}/${NAME}"

echo "保存到前缀: ${PREFIX}"
# Foxy: map_saver；较新发行版多为 map_saver_cli
if ros2 pkg executables nav2_map_server 2>/dev/null | grep -q map_saver_cli; then
  exec ros2 run nav2_map_server map_saver_cli -f "${PREFIX}"
fi
exec ros2 run nav2_map_server map_saver -f "${PREFIX}"
