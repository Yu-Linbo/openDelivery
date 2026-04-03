# Copyright (c) 2018 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Fork of nav2_bringup/navigation_launch.py (Foxy): remap `map` subscription to a namespaced
# OccupancyGrid topic (e.g. /<robot>/map or /<robot>/mapping). No relay to /map.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_subscribe_transient_local = LaunchConfiguration("map_subscribe_transient_local")
    occupancy_grid_topic = LaunchConfiguration("occupancy_grid_topic")

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "recoveries_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    remappings = [
        ("/tf", "/tf"),
        ("/tf_static", "/tf_static"),
        ("map", occupancy_grid_topic),
    ]

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "default_bt_xml_filename": default_bt_xml_filename,
        "autostart": autostart,
        "map_subscribe_transient_local": map_subscribe_transient_local,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation clock if true"
            ),
            DeclareLaunchArgument(
                "autostart", default_value="true", description="Automatically startup the nav2 stack"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
                description="Full path to the ROS 2 parameters file to use",
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=os.path.join(
                    get_package_share_directory("nav2_bt_navigator"),
                    "behavior_trees",
                    "navigate_w_replanning_and_recovery.xml",
                ),
                description="Full path to the behavior tree XML file to use",
            ),
            DeclareLaunchArgument(
                "map_subscribe_transient_local",
                default_value="false",
                description="Whether to set the map subscriber QoS to transient local",
            ),
            DeclareLaunchArgument(
                "occupancy_grid_topic",
                description=(
                    "Absolute OccupancyGrid topic for global costmap (e.g. /robot2/map or /robot2/mapping)"
                ),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_recoveries",
                executable="recoveries_server",
                name="recoveries_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
