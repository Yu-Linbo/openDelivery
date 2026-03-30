"""fake_pub 演示节点（与 `simulate` Gazebo 包无关）。安装到 bringup_launch/fake/fake_pub.launch.py；命名空间策略与 heartbeat.launch.py 一致。"""

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
                description="When namespace is not set, also selects namespace (PushRosNamespace)",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value=LaunchConfiguration("robot_name"),
                description="ROS 2 namespace for this fake_pub instance",
            ),
            DeclareLaunchArgument("current_map", default_value="nh_102"),
            GroupAction(
                actions=[
                    PushRosNamespace(LaunchConfiguration("namespace")),
                    Node(
                        package="open_delivery_system",
                        executable="fake_pub_node",
                        name="fake_pub",
                        output="screen",
                        parameters=[
                            {
                                "robot_name": LaunchConfiguration("namespace"),
                                "current_map": LaunchConfiguration("current_map"),
                            }
                        ],
                    ),
                ]
            ),
        ]
    )
