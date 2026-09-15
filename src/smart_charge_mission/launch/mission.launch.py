"""启动充电任务状态机。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("smart_charge_mission")
    nav_share = get_package_share_directory("smart_charge_navigation")
    waypoints_file = os.path.join(nav_share, "config", "waypoints.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mission_params_file",
            default_value=os.path.join(pkg_share, "config", "mission_params.yaml"),
        ),
        Node(
            package="smart_charge_mission",
            executable="charge_mission_node",
            name="charge_mission",
            output="screen",
            parameters=[
                LaunchConfiguration("mission_params_file"),
                {"waypoints_file": waypoints_file},
            ],
        ),
    ])
