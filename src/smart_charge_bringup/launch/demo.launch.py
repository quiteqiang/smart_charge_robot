"""向后兼容别名：demo.launch.py == full_demo.launch.py。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    src = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("smart_charge_bringup"),
                     "launch", "full_demo.launch.py"))
    return LaunchDescription([IncludeLaunchDescription(src)])
