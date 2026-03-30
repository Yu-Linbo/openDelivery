"""Debug: keep in sync with openDelivery/params/launch/system/heartbeat.launch.py.

Isolation model: PushRosNamespace so publisher/service use relative names and appear as
/<namespace>/robot_status and /<namespace>/set_heartbeat_params. Multiple instances:
repeat the GroupAction with a different namespace (or include this launch twice).
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
            DeclareLaunchArgument("robot_status", default_value="normal"),
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
                                "current_map": LaunchConfiguration("current_map"),
                                "robot_status": LaunchConfiguration("robot_status"),
                                "publish_rate": LaunchConfiguration("publish_rate"),
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
