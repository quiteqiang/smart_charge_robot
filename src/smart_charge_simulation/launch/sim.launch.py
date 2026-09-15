"""启动矿卡最小仿真器（解析世界模型见 config/world.yaml）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("smart_charge_simulation")
    world_file = os.path.join(pkg_share, "config", "world.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "sim_params_file",
            default_value=os.path.join(pkg_share, "config", "sim_params.yaml"),
            description="仿真器参数文件",
        ),
        Node(
            package="smart_charge_simulation",
            executable="mining_truck_sim_node",
            name="mining_truck_sim",
            output="screen",
            parameters=[
                LaunchConfiguration("sim_params_file"),
                {"world_file": world_file},
            ],
        ),
    ])
