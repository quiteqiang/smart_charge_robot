"""一键启动完整演示：仿真 + 描述 + 导航 + 电池 + 泊靠 + 任务 (+RViz/录包，可关)。

用法：
  ros2 launch smart_charge_bringup full_demo.launch.py
  ros2 launch smart_charge_bringup full_demo.launch.py use_rviz:=false record_bag:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg: str, launch_file: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), "launch", launch_file))


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    record_bag = LaunchConfiguration("record_bag")
    nav_share = get_package_share_directory("smart_charge_navigation")

    ld = LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="false"),
    ])

    ld.add_action(IncludeLaunchDescription(include("smart_charge_simulation", "sim.launch.py")))
    ld.add_action(IncludeLaunchDescription(include("smart_charge_base", "description.launch.py")))
    ld.add_action(IncludeLaunchDescription(include("smart_charge_battery", "battery.launch.py")))
    ld.add_action(IncludeLaunchDescription(include("smart_charge_docking", "docking.launch.py")))
    ld.add_action(IncludeLaunchDescription(include("smart_charge_mission", "mission.launch.py")))
    ld.add_action(IncludeLaunchDescription(include("smart_charge_navigation", "navigation.launch.py")))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(nav_share, "rviz", "smart_charge.rviz")],
        condition=IfCondition(use_rviz),
    ))

    # rosbag2 录制（相对路径写入工作区 bags/）
    ld.add_action(Node(
        package="rosbag2_transport",
        executable="recorder",
        name="rosbag_recorder",
        output="screen",
        arguments=["-o", "bags/demo_run",
                   "/scan", "/odom", "/imu", "/cmd_vel",
                   "/battery_state", "/mission_state", "/docking_success",
                   "/docking_status", "/charging_active", "/dock_contact",
                   "/tf", "/tf_static"],
        condition=IfCondition(record_bag),
    ))
    return ld
