"""health_monitor + task_manager (same namespace as heartbeat; normally included from heartbeat.launch.py)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
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
            DeclareLaunchArgument(
                "require_nav2",
                default_value="false",
                description=(
                    "When 'true', bt_navigator must be running before health_monitor "
                    "advances to the localizing phase. Set to 'true' when launching the "
                    "full nav stack alongside the manager."
                ),
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    # Without Nav2: only gate on heartbeat being present.
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        condition=UnlessCondition(LaunchConfiguration("require_nav2")),
                        parameters=[
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat"],
                            }
                        ],
                    ),
                    # With Nav2 (require_nav2:=true): also gate on bt_navigator being active
                    # before advancing robot_status to localizing.
                    Node(
                        package="manager",
                        executable="health_monitor_node",
                        name="health_monitor",
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("require_nav2")),
                        parameters=[
                            {
                                "localization_pose_topic": LaunchConfiguration(
                                    "localization_pose_topic"
                                ),
                                "required_nodes": ["heartbeat", "bt_navigator"],
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
