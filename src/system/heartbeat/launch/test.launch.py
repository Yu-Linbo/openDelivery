"""Debug: starts the same stack as heartbeat.launch.py with a log line."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("heartbeat")
    return LaunchDescription(
        [
            LogInfo(msg="[heartbeat/test.launch.py] include heartbeat.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "heartbeat.launch.py")
                )
            ),
        ]
    )
