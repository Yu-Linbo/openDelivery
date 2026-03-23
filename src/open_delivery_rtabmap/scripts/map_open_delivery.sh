#!/usr/bin/env bash
set -eo pipefail
# 建图：2D 雷达 + /odom，可选 IMU（launch 参数 imu_topic:=/your/imu）
# 依赖：已安装 ros-foxy-rtabmap / ros-foxy-rtabmap-ros（提供 rtabmap_slam 与依赖）
# 用法：在 openDelivery 目录执行 ./start_rtabmap_mapping.sh，或手动：
#   export OPENDELIVERY_ROOT=/path/to/openDelivery
#   ros2 run open_delivery_rtabmap map_open_delivery.sh

: "${OPENDELIVERY_ROOT:?Set OPENDELIVERY_ROOT to the openDelivery directory (contains map/ and install/).}"

ROS_DISTRO="${ROS_DISTRO:-foxy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${OPENDELIVERY_ROOT}/install/setup.bash"
export OPENDELIVERY_MAP_DIR="${OPENDELIVERY_MAP_DIR:-${OPENDELIVERY_ROOT}/map}"

exec ros2 launch open_delivery_rtabmap mapping.launch.py "$@"
