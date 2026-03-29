"""Debug: same as `ros2 launch open_delivery_system startup.launch.py` via IncludeLaunchDescription."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = get_package_share_directory("open_delivery_system")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(root, "startup.launch.py"))
            ),
        ]
    )
