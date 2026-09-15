"""导航 bringup：map_server + AMCL + 裁剪版 Nav2（低内存环境）。

启动时把包内地图的绝对路径注入参数文件副本（占位符 __MAP_YAML__），
避免在 nav2_params.yaml 里写死路径。
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _materialize_params(pkg_share: str) -> str:
    src = os.path.join(pkg_share, "config", "nav2_params.yaml")
    map_yaml = os.path.join(pkg_share, "config", "site.yaml")
    bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"), "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml")
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    content = content.replace("__MAP_YAML__", map_yaml)
    content = content.replace("__BT_XML__", bt_xml)
    fd, dst = tempfile.mkstemp(prefix="smart_charge_nav2_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(content)
    return dst


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("smart_charge_navigation")
    params_file = LaunchConfiguration("nav2_params_file")
    autostart = LaunchConfiguration("autostart", default="true")

    default_params = _materialize_params(pkg_share)

    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": autostart,
                "node_names": ["map_server", "amcl"],
            }],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": autostart,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }],
        ),
    ]

    ld = LaunchDescription([
        DeclareLaunchArgument("nav2_params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
    ])
    for n in nodes:
        ld.add_action(n)
    return ld
