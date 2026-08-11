from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("log_root", default_value="log_bag"),
            DeclareLaunchArgument("max_bag_bytes", default_value="104857600"),
            Node(
                package="log_bag",
                executable="robot_log_recorder",
                name="robot_log_recorder",
                namespace=LaunchConfiguration("robot_name"),
                output="screen",
                arguments=[
                    "--robot-name",
                    LaunchConfiguration("robot_name"),
                    "--root",
                    LaunchConfiguration("log_root"),
                    "--max-bag-bytes",
                    LaunchConfiguration("max_bag_bytes"),
                ],
            ),
        ]
    )
