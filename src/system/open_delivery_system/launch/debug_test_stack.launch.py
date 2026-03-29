"""Debug: full stack with a log line (same graph as startup.launch.py)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = get_package_share_directory("open_delivery_system")
    return LaunchDescription(
        [
            LogInfo(msg="[debug_test_stack.launch.py] including startup.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(root, "startup.launch.py"))
            ),
        ]
    )
