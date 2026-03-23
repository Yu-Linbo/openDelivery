#!/usr/bin/env bash
set -eo pipefail
# 不用 set -u：/opt/ros/*/setup.bash 会引用未设置的可选变量（如 AMENT_TRACE_SETUP_FILES）
# 一键启动 RTAB-Map 建图（Foxy）：2D 激光 + 里程计，地图与数据库在 openDelivery/map
# 可选环境变量: ROS_DISTRO (默认 foxy)
# 可选参数传递给 launch，例如: imu_topic:=/imu/data  scan_topic:=/scan  odom_topic:=/odom

cd "$(dirname "$0")"
export OPENDELIVERY_ROOT="$(pwd)"
export OPENDELIVERY_MAP_DIR="${OPENDELIVERY_ROOT}/map"

ROS_DISTRO="${ROS_DISTRO:-foxy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ ! -f install/setup.bash ]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "[openDelivery] 未找到 colcon，请先安装: sudo apt install python3-colcon-common-extensions" >&2
    exit 1
  fi
  echo "[openDelivery] 首次编译工作空间..."
  colcon build --symlink-install --packages-select open_delivery_rtabmap
fi

source "${OPENDELIVERY_ROOT}/install/setup.bash"

echo "[openDelivery] OPENDELIVERY_MAP_DIR=${OPENDELIVERY_MAP_DIR}"
echo "[openDelivery] 另开终端保存栅格地图: ros2 run open_delivery_rtabmap save_map_open_delivery.sh <名称>"
exec ros2 launch open_delivery_rtabmap mapping.launch.py "$@"
