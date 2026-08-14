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
            DeclareLaunchArgument("robot_model", default_value="OP1"),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            DeclareLaunchArgument("robot_status", default_value="initializing"),
            DeclareLaunchArgument("sim_mode", default_value="sim"),
            DeclareLaunchArgument("mapping_mode", default_value="false"),
            DeclareLaunchArgument("publish_rate", default_value="2.0"),
            DeclareLaunchArgument("control_status", default_value="AUTO"),
            DeclareLaunchArgument("log_root", default_value="log_bag"),
            DeclareLaunchArgument("max_bag_bytes", default_value="5242880"),
            DeclareLaunchArgument("enable_fake_pub", default_value="true"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg, "bringup_launch", "system", "heartbeat.launch.py"]
                    )
                ),
                launch_arguments=[
                    ("robot_name", LaunchConfiguration("robot_name")),
                    ("robot_model", LaunchConfiguration("robot_model")),
                    ("current_map", LaunchConfiguration("current_map")),
                    ("robot_status", LaunchConfiguration("robot_status")),
                    ("sim_mode", LaunchConfiguration("sim_mode")),
                    ("mapping_mode", LaunchConfiguration("mapping_mode")),
                    ("publish_rate", LaunchConfiguration("publish_rate")),
                    ("control_status", LaunchConfiguration("control_status")),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("chassis_state_machine"), "launch", "chassis_state_machine.launch.py"]
                    )
                ),
                launch_arguments=[("robot_name", LaunchConfiguration("robot_name"))],
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
