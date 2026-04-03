# Copyright (c) 2018 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Navigation2 stack: costmap subscribes via remap to /<robot>/map or /<robot>/mapping
# (see grid_mode). No relay to /map. TF from SLAM.

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def _as_bool(text: str) -> bool:
    return (text or "").strip().lower() in ("true", "1", "yes", "on")


def _materialize_params(robot_name: str, pkg_share: str, occ_topic: str) -> str:
    template_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()
    text = text.replace("@@ROBOT@@", robot_name)
    text = text.replace("@@OCC_GRID_TOPIC@@", occ_topic)
    data = yaml.safe_load(text)
    # Top-level keys must be node names; RewrittenYaml adds the namespace wrapper.
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="nav2_params_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    return path


def _launch_setup(context, *_args, **_kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "robot2"
    grid_mode = LaunchConfiguration("grid_mode").perform(context).strip().lower()
    use_sim = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_sim_str = "true" if use_sim else "false"

    occ_topic = (
        f"/{robot_name}/mapping" if grid_mode == "mapping" else f"/{robot_name}/map"
    )

    pkg_nav = get_package_share_directory("nav_bringup")
    params_path = _materialize_params(robot_name, pkg_nav, occ_topic)
    bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_w_replanning_and_recovery.xml",
    )

    return [
        GroupAction(
            [
                PushRosNamespace(robot_name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav, "launch", "navigation_namespaced.launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot_name,
                        "use_sim_time": use_sim_str,
                        "autostart": "true",
                        "params_file": params_path,
                        "default_bt_xml_filename": bt_xml,
                        "map_subscribe_transient_local": "true",
                        "occupancy_grid_topic": occ_topic,
                    }.items(),
                ),
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="robot2",
                description="ROS namespace for Nav2 nodes (match slam_bringup / robot TF prefix)",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Must match Gazebo / SLAM use_sim_time",
            ),
            DeclareLaunchArgument(
                "grid_mode",
                default_value="localize",
                description="localize: costmap uses /<robot>/map; mapping: uses /<robot>/mapping",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
