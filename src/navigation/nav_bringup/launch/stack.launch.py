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


def _materialize_params(
    robot_name: str,
    pkg_share: str,
    occ_topic: str,
    *,
    global_rolling_window: str,
    global_window_width: str,
    global_window_height: str,
    track_unknown_space: str,
    unknown_cost_value: str,
    allow_unknown: str,
    local_track_unknown_space: str,
) -> str:
    template_path = os.path.join(pkg_share, "config", "nav2_params.yaml")
    with open(template_path, "r", encoding="utf-8") as f:
        text = f.read()
    text = text.replace("@@ROBOT@@", robot_name)
    text = text.replace("@@OCC_GRID_TOPIC@@", occ_topic)
    text = text.replace("@@GLOBAL_ROLLING_WINDOW@@", global_rolling_window)
    text = text.replace("@@GLOBAL_WINDOW_WIDTH@@", global_window_width)
    text = text.replace("@@GLOBAL_WINDOW_HEIGHT@@", global_window_height)
    text = text.replace("@@TRACK_UNKNOWN_SPACE@@", track_unknown_space)
    text = text.replace("@@UNKNOWN_COST_VALUE@@", unknown_cost_value)
    text = text.replace("@@ALLOW_UNKNOWN@@", allow_unknown)
    text = text.replace("@@LOCAL_TRACK_UNKNOWN_SPACE@@", local_track_unknown_space)
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
    autostart = LaunchConfiguration("autostart").perform(context).strip() or "true"

    occ_topic = (
        f"/{robot_name}/mapping" if grid_mode == "mapping" else f"/{robot_name}/map"
    )
    # slam mapping stream is usually volatile (non-transient), while map_server is transient local.
    map_subscribe_transient_local = "false" if grid_mode == "mapping" else "true"
    if grid_mode == "mapping":
        # Mapping mode: keep robot inside global costmap even after manual teleport.
        global_rolling_window = "true"
        global_window_width = "80"
        global_window_height = "80"
        # SLAM frontiers are mostly unknown (-1): treat unknown as FREE on global + local
        # costmaps so paths exist and the controller is not stuck in "rotate only".
        track_unknown_space = "false"
        unknown_cost_value = "255"
        allow_unknown = "true"
        local_track_unknown_space = "false"
    else:
        # Localization mode: fixed global map; respect unknown boundaries on the map.
        global_rolling_window = "false"
        global_window_width = "0"
        global_window_height = "0"
        track_unknown_space = "true"
        unknown_cost_value = "255"
        allow_unknown = "true"
        local_track_unknown_space = "true"

    pkg_nav = get_package_share_directory("nav_bringup")
    params_path = _materialize_params(
        robot_name,
        pkg_nav,
        occ_topic,
        global_rolling_window=global_rolling_window,
        global_window_width=global_window_width,
        global_window_height=global_window_height,
        track_unknown_space=track_unknown_space,
        unknown_cost_value=unknown_cost_value,
        allow_unknown=allow_unknown,
        local_track_unknown_space=local_track_unknown_space,
    )
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
                        "autostart": autostart,
                        "params_file": params_path,
                        "default_bt_xml_filename": bt_xml,
                        "map_subscribe_transient_local": map_subscribe_transient_local,
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
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Nav2 lifecycle manager autostart flag",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
