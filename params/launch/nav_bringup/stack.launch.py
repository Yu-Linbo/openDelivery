"""Production Nav2 stack launch; installed by the nav_bringup package."""

# Copyright (c) 2018 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Navigation2 stack: costmap subscribes via remap to /<robot>/map or /<robot>/mapping
# (see grid_mode). No relay to /map. TF from SLAM.

import json
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


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
    map_subscribe_transient_local: str,
    robot_settings_file: str = "",
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
    text = text.replace("@@MAP_SUBSCRIBE_TRANSIENT_LOCAL@@", map_subscribe_transient_local)
    data = yaml.safe_load(text)
    settings = _load_robot_settings(robot_settings_file, robot_name)
    if settings:
        controller = data["controller_server"]["ros__parameters"]["FollowPath"]
        controller["max_vel_x"] = settings["max_linear_speed"]
        controller["max_speed_xy"] = settings["max_linear_speed"]
        controller["max_vel_theta"] = settings["max_angular_speed"]
        data["recoveries_server"]["ros__parameters"]["max_rotational_vel"] = settings[
            "max_angular_speed"
        ]
        for costmap_name in ("local_costmap", "global_costmap"):
            data[costmap_name][costmap_name]["ros__parameters"]["inflation_layer"][
                "inflation_radius"
            ] = settings["inflation_radius"]
    # Top-level keys must be node names; RewrittenYaml adds the namespace wrapper.
    fd, path = tempfile.mkstemp(suffix=".yaml", prefix="nav2_params_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    return path


def _load_robot_settings(path: str, robot_name: str):
    """Read validated Web settings without making launch depend on the backend package."""
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = json.load(stream)
        entry = (raw.get("robots") or {}).get(robot_name) or {}
        settings = entry.get("settings") or {}
        values = {
            "max_linear_speed": float(settings["max_linear_speed"]),
            "max_angular_speed": float(settings["max_angular_speed"]),
            "inflation_radius": float(settings["inflation_radius"]),
        }
        limits = {
            "max_linear_speed": (0.05, 2.0),
            "max_angular_speed": (0.1, 3.0),
            "inflation_radius": (0.22, 3.0),
        }
        if not all(math.isfinite(value) and limits[name][0] <= value <= limits[name][1]
                   for name, value in values.items()):
            return None
        return values
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _launch_setup(context, *_args, **_kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "robot2"
    grid_mode = LaunchConfiguration("grid_mode").perform(context).strip().lower()
    use_sim = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    use_sim_str = "true" if use_sim else "false"
    autostart = LaunchConfiguration("autostart").perform(context).strip() or "true"
    robot_settings_file = LaunchConfiguration("robot_settings_file").perform(context).strip()

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
        map_subscribe_transient_local=map_subscribe_transient_local,
        robot_settings_file=robot_settings_file,
    )
    bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_w_replanning_and_recovery.xml",
    )
    navigation_namespace = f"{robot_name}/navigation"

    return [
        GroupAction(
            [
                PushRosNamespace(navigation_namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav, "launch", "navigation_namespaced.launch.py")
                    ),
                    launch_arguments={
                        "namespace": navigation_namespace,
                        "robot_name": robot_name,
                        "use_sim_time": use_sim_str,
                        "autostart": autostart,
                        "params_file": params_path,
                        "default_bt_xml_filename": bt_xml,
                        "map_subscribe_transient_local": map_subscribe_transient_local,
                        "occupancy_grid_topic": occ_topic,
                    }.items(),
                ),
                Node(
                    package="navigation_tasks",
                    executable="navigation_task_node",
                    name="task_executor",
                    output="screen",
                    parameters=[
                        {
                            "robot_name": robot_name,
                            "map_frame": "map",
                            "action_server_wait_sec": 15.0,
                            "nav2_goal_retry_count": 2,
                            "nav2_goal_retry_delay_sec": 1.0,
                            "nav2_goal_response_timeout_sec": 10.0,
                            "nav2_feedback_timeout_sec": 20.0,
                            "use_sim_time": use_sim,
                        }
                    ],
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
                description="ROS namespace for Nav2 nodes (match robot TF prefix)",
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
            DeclareLaunchArgument(
                "robot_settings_file",
                default_value="",
                description="Per-robot settings JSON written by the OpenDelivery Web backend",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
