"""Debug: starts the same stack as simulate.launch.py with a log line."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("simulate")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="drawn_model.world",
                description="Gazebo world file forwarded to simulate.launch.py.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="sim_robot",
                description="Forwarded to simulate.launch.py (default namespace base).",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value=LaunchConfiguration("robot_name"),
                description="Forwarded to simulate.launch.py.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Forwarded to simulate.launch.py.",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Forwarded to simulate.launch.py.",
            ),
            DeclareLaunchArgument(
                "start_gzweb",
                default_value="false",
                description="Forwarded to simulate.launch.py; requires GzWeb checkout path.",
            ),
            DeclareLaunchArgument(
                "gzweb_root",
                default_value="",
                description="Forwarded to simulate.launch.py (or set GZWEB_ROOT).",
            ),
            DeclareLaunchArgument(
                "gzweb_port",
                default_value="8080",
                description="Forwarded to simulate.launch.py.",
            ),
            DeclareLaunchArgument(
                "gzweb_start_delay",
                default_value="8.0",
                description="Forwarded to simulate.launch.py (seconds after launch).",
            ),
            LogInfo(msg="[simulate/test.launch.py] include simulate.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "simulate.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "namespace": LaunchConfiguration("namespace"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "start_gazebo": LaunchConfiguration("start_gazebo"),
                    "start_gzweb": LaunchConfiguration("start_gzweb"),
                    "gzweb_root": LaunchConfiguration("gzweb_root"),
                    "gzweb_port": LaunchConfiguration("gzweb_port"),
                    "gzweb_start_delay": LaunchConfiguration("gzweb_start_delay"),
                }.items(),
            ),
        ]
    )
