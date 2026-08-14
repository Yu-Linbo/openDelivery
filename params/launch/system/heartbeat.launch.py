"""Bring up robot_status heartbeat only (install: bringup_launch/system/heartbeat.launch.py).

Manager nodes are started from sim_bringup.sh after heartbeat activate, not here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="robot2",
                description="Legacy: when namespace is not set, also selects namespace (PushRosNamespace)",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value=LaunchConfiguration("robot_name"),
                description="ROS 2 namespace for this instance; overrides isolation prefix independent of other args",
            ),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            DeclareLaunchArgument("robot_model", default_value="OP1"),
            DeclareLaunchArgument(
                "robot_status",
                default_value="initializing",
                description="RobotStatus.robot_status string: initializing|localizing|localization_lost|ready|shutdown.",
            ),
            DeclareLaunchArgument(
                "task_status",
                default_value="idle",
                description="RobotStatus.task_status string: idle|mapping|delivery|cleaning|patrolling.",
            ),
            DeclareLaunchArgument(
                "control_status",
                default_value="AUTO",
                description="RobotStatus.control_status: AUTO|JOY|MANUAL.",
            ),
            DeclareLaunchArgument("sim_mode", default_value="sim"),
            DeclareLaunchArgument(
                "mapping_mode",
                default_value="false",
                description="If true, publish task_status=mapping (current_map uses <robot>_mapping).",
            ),
            DeclareLaunchArgument("publish_rate", default_value="2.0"),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    Node(
                        package="heartbeat",
                        executable="heartbeat_node",
                        name="heartbeat",
                        output="screen",
                        parameters=[
                            {
                                "robot_name": LaunchConfiguration("namespace"),
                                "robot_model": LaunchConfiguration("robot_model"),
                                "current_map": LaunchConfiguration("current_map"),
                                "robot_status": LaunchConfiguration("robot_status"),
                                "task_status": LaunchConfiguration("task_status"),
                                "control_status": LaunchConfiguration("control_status"),
                                "sim_mode": LaunchConfiguration("sim_mode"),
                                "mapping_mode": LaunchConfiguration("mapping_mode"),
                                "publish_rate": LaunchConfiguration("publish_rate"),
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
