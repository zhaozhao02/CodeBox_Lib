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
   
    aruco_pose = Node(
        package='aruco_pose',
        executable='aruco_pose_node',
        parameters=[
            {'calibration_file': '/home/khadas/Workspace/tracking_ws/store/calibration/calibration_params.yml'},
        ],
        arguments=['-v=/home/khadas/Workspace/tracking_ws/store/video/1080p_30fps_Raw_4.mp4',
                   '-l=0.08',
                   '-d=16',
                   ],
    )


    return LaunchDescription([
        aruco_pose,
    ])
