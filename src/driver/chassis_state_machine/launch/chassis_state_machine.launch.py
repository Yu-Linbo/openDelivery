from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot2"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.5"),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("robot_name")),
                    Node(
                        package="chassis_state_machine",
                        executable="chassis_state_machine_node",
                        output="screen",
                        parameters=[
                            {"command_timeout_sec": LaunchConfiguration("command_timeout_sec")}
                        ],
                    ),
                ]
            ),
        ]
    )
