"""Top-level stack: includes bringup launches and always-on per-robot logging."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("system")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            DeclareLaunchArgument("robot_status", default_value="0"),
            DeclareLaunchArgument("sim_mode", default_value="sim"),
            DeclareLaunchArgument("mapping_mode", default_value="false"),
            DeclareLaunchArgument("publish_rate", default_value="2.0"),
            DeclareLaunchArgument("log_root", default_value="log_bag"),
            DeclareLaunchArgument("max_bag_bytes", default_value="104857600"),
            DeclareLaunchArgument("enable_fake_pub", default_value="true"),
            Node(
                package="log_bag",
                executable="robot_log_recorder",
                name="log_bag",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg, "bringup_launch", "system", "heartbeat.launch.py"]
                    )
                ),
                launch_arguments=[
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("current_map", LaunchConfiguration("current_map")),
                    ("robot_status", LaunchConfiguration("robot_status")),
                    ("sim_mode", LaunchConfiguration("sim_mode")),
                    ("mapping_mode", LaunchConfiguration("mapping_mode")),
                    ("publish_rate", LaunchConfiguration("publish_rate")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg, "bringup_launch", "fake", "fake_pub.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("enable_fake_pub")),
                launch_arguments=[
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("current_map", LaunchConfiguration("current_map")),
                ],
            ),
        ]
    )
