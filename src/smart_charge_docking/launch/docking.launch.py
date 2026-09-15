from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('smart_charge_docking')
    return LaunchDescription([
        DeclareLaunchArgument(
            'dock_params_file',
            default_value=os.path.join(pkg_share, 'config', 'dock_controller.yaml'),
        ),
        Node(
            package='smart_charge_docking',
            executable='dock_controller_node',
            name='dock_controller',
            output='screen',
            parameters=[LaunchConfiguration('dock_params_file')],
        ),
    ])
