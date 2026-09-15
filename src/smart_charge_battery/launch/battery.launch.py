from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('smart_charge_battery')
    return LaunchDescription([
        DeclareLaunchArgument(
            'battery_params_file',
            default_value=os.path.join(pkg_share, 'config', 'battery_params.yaml'),
        ),
        Node(
            package='smart_charge_battery',
            executable='battery_simulator_node',
            name='battery_simulator',
            output='screen',
            parameters=[LaunchConfiguration('battery_params_file')],
        ),
    ])
