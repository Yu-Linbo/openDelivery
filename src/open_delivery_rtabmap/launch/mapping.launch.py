#!/usr/bin/env python3
"""RTAB-Map mapping: 2D /scan + /odom, optional /imu (see imu_topic arg)."""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_rtabmap_slam_executable(prefix: str):
    """Foxy 等发行版常安装为 lib/rtabmap_slam/rtabmap，而非 rtabmap_slam。"""
    lib_pkg = os.path.join(prefix, 'lib', 'rtabmap_slam')
    lib_root = os.path.join(prefix, 'lib')
    search_dirs = [d for d in (lib_pkg, lib_root) if os.path.isdir(d)]
    for name in ('rtabmap', 'rtabmap_slam', 'rtabmap_slam_node'):
        for d in search_dirs:
            path = os.path.join(d, name)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return name
    if os.path.isdir(lib_pkg):
        for f in sorted(os.listdir(lib_pkg)):
            if f.endswith('.so'):
                continue
            path = os.path.join(lib_pkg, f)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return f
    return None


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('open_delivery_rtabmap')
    params_file = os.path.join(pkg_share, 'config', 'rtabmap_2d_laser.yaml')

    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    map_dir = LaunchConfiguration('map_dir').perform(context)
    exe_arg = LaunchConfiguration('slam_executable').perform(context).strip()

    db_dir = os.path.join(map_dir, 'rtabmap')
    os.makedirs(db_dir, exist_ok=True)
    db_path = os.path.join(db_dir, 'rtabmap.db')

    extra_params = []
    if imu_topic:
        extra_params.append({'imu_topic': imu_topic, 'wait_imu_to_init': True})

    try:
        rtab_prefix = get_package_prefix('rtabmap_slam')
    except Exception as ex:  # noqa: BLE001 — ament 版本差异
        raise RuntimeError(
            "未找到 ROS 包 rtabmap_slam。请安装: sudo apt install ros-foxy-rtabmap-slam"
        ) from ex

    if exe_arg and exe_arg != 'auto':
        slam_exe = exe_arg
    else:
        slam_exe = _resolve_rtabmap_slam_executable(rtab_prefix)

    if not slam_exe:
        raise RuntimeError(
            "在 rtabmap_slam 安装目录中未找到可执行文件。"
            "请执行: ros2 pkg executables rtabmap_slam\n"
            "若列表中有其它名称，请启动时指定: slam_executable:=<名称>"
        )

    if exe_arg == 'auto' or not exe_arg:
        print(
            f'[open_delivery_rtabmap] rtabmap_slam executable: {slam_exe}',
            file=sys.stderr,
        )

    node = Node(
        package='rtabmap_slam',
        executable=slam_exe,
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            *extra_params,
            {'database_path': db_path},
        ],
        remappings=[
            ('scan', scan_topic),
            ('odom', odom_topic),
        ],
    )
    return [node]


def generate_launch_description():
    default_map_dir = os.environ.get(
        'OPENDELIVERY_MAP_DIR',
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'map')
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map_dir',
                default_value=default_map_dir,
                description='OpenDelivery map root (PGM/YAML + rtabmap subfolder for .db).',
            ),
            DeclareLaunchArgument(
                'scan_topic',
                default_value='scan',
                description='sensor_msgs/LaserScan topic (remapped to rtabmap scan).',
            ),
            DeclareLaunchArgument(
                'odom_topic',
                default_value='odom',
                description='nav_msgs/Odometry for motion prediction.',
            ),
            DeclareLaunchArgument(
                'imu_topic',
                default_value='',
                description='sensor_msgs/Imu topic; leave empty to use odom-only.',
            ),
            DeclareLaunchArgument(
                'slam_executable',
                default_value='auto',
                description="rtabmap_slam 包内可执行文件名；Foxy 常见为 rtabmap。'auto' 则自动探测。",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
