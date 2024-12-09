from http.server import executable
import os
from struct import pack
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   
    # 启动激光雷达
    aruco_calibration = Node(
        package='aruco_calibration',
        executable='aruco_calibration_node',
        arguments=['-v=/home/khadas/roaf3d_ws_loop/store/video/KS2A123-2.0_2.mp4',
                   '-d=16',
                   '-dp=/home/khadas/roaf3d_ws_loop/store/calibration/detector_params.yml',
                   '-h=2',
                   '-w=4',
                   '-l=0.05',
                   '-s=0.02486',
                   '-o=/home/khadas/roaf3d_ws_loop/store/calibration/KS2A123-2.0.yml'],
    )


    return LaunchDescription([
        aruco_calibration,
    ])
