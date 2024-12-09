import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取aruco_tracking包的share目录
    config_yaml = Path(get_package_share_directory('aruco_tracking'), 'config', 'px4_config.yaml')
    pluginlists_yaml = Path(get_package_share_directory('aruco_tracking'), 'config', 'px4_pluginlists.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyTHS1:921600',
            description='FCU URL'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS URL'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),
        DeclareLaunchArgument(
            'pluginlists_yaml',
            default_value=str(pluginlists_yaml),
            description='Path to pluginlists yaml file'
        ),
        DeclareLaunchArgument(
            'config_yaml',
            default_value=str(config_yaml),
            description='Path to config yaml file'
        ),
        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Log output configuration'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='FCU protocol version'
        ),
        DeclareLaunchArgument(
            'respawn_mavros',
            default_value='false',
            description='Respawn MAVROS node on failure'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='mavros',
            description='Namespace for the MAVROS node'
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output=LaunchConfiguration('log_output'),
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'tgt_system': LaunchConfiguration('tgt_system'),
                'tgt_component': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            }, LaunchConfiguration('config_yaml'), LaunchConfiguration('pluginlists_yaml')],
            respawn=LaunchConfiguration('respawn_mavros')
        )
    ])
