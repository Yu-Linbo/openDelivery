"""Optional: health_monitor + task_manager under the same namespace as heartbeat."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="robot2",
                description="PushRosNamespace for this robot (match heartbeat).",
            ),
            DeclareLaunchArgument(
                "localization_pose_topic",
                default_value="amcl_pose",
                description="Empty string disables pose-based ready transition.",
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        parameters=[
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="manager",
                        executable="task_manager_node",
                        name="task_manager",
                        output="screen",
                    ),
                ]
            ),
        ]
    )
