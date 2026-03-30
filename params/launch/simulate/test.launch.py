"""Debug: include simulate.launch.py（installed to bringup_launch/simulate/test.launch.py；源码镜像见 src/simulate/simulate/launch/test.launch.py）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("simulate")
    return LaunchDescription(
        [
            LogInfo(msg="[simulate/test.launch.py] include simulate.launch.py"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "simulate.launch.py")
                )
            ),
        ]
    )
