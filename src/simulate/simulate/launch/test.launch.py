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
            LogInfo(msg="[simulate/test.launch.py] include simulate.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "simulate.launch.py")
                ),
                launch_arguments={"world": LaunchConfiguration("world")}.items(),
            ),
        ]
    )
