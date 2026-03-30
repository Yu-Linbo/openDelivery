"""Top-level stack: includes bringup launches under bringup_launch/ (e.g. system/*.launch.py, fake/*.launch.py)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("open_delivery_system")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            DeclareLaunchArgument("robot_status", default_value="normal"),
            DeclareLaunchArgument("publish_rate", default_value="2.0"),
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
                    ("publish_rate", LaunchConfiguration("publish_rate")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg, "bringup_launch", "fake", "fake_pub.launch.py"]
                    )
                ),
                launch_arguments=[
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("current_map", LaunchConfiguration("current_map")),
                ],
            ),
        ]
    )
