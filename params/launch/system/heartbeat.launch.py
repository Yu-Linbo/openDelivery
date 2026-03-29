"""Bring up robot_status heartbeat node (package: src/system/heartbeat; installed to bringup_launch/system/heartbeat.launch.py)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            DeclareLaunchArgument("robot_status", default_value="normal"),
            DeclareLaunchArgument("publish_rate", default_value="2.0"),
            Node(
                package="heartbeat",
                executable="heartbeat_node",
                name="heartbeat",
                output="screen",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "current_map": LaunchConfiguration("current_map"),
                        "robot_status": LaunchConfiguration("robot_status"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                    }
                ],
            ),
        ]
    )
