"""Simulation: fake_pub (mirrors src/simulate/simulate; installed to bringup_launch/simulate/simulate/). Pair with bringup_launch/system/heartbeat.launch.py for heartbeat-only bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            Node(
                package="open_delivery_system",
                executable="fake_pub_node",
                name="fake_pub",
                output="screen",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "current_map": LaunchConfiguration("current_map"),
                    }
                ],
            ),
        ]
    )
