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
            {'calibration_file': '/home/khadas/roaf3d_ws_loop/store/calibration/calibration_1080p_params.yml'},
            {'device': '/dev/video0'},
            {'width': 1920},
            {'height': 1080},
            {'framerate': 30},
        ],
        arguments=['-v=0',
                   '-l=0.08',
                   '-d=16',
                    '-dp=/home/khadas/roaf3d_ws_loop/store/calibration/detector_params.yml',
                   ],
    )


    usb_cam = Node(
        package='aruco_pose',
        executable='usb_cam_node',
        parameters=[
            {'device': '/dev/video0'},
            {'width': 1600},
            {'height': 1200},
            {'framerate': 60},
        ],
    )


    aruco_detector = Node(
        package='aruco_pose',
        executable='aruco_detector_node',
        parameters=[
            {'calibration_file': '/home/khadas/Workspace/tracking_ws/store/calibration/calibration_1080p_params.yml'},
            {'detect_hz': 5},
            {'show_pic': True},
        ],
        arguments=['-v=0',
                   '-l=0.08',
                   '-d=16',
                    '-dp=/home/khadas/Workspace/tracking_ws/store/calibration/detector_params.yml',
                   ],
    )

    aruco_detector_mul = Node(
        package='aruco_pose',
        executable='aruco_detector_mul_node',
        parameters=[
            {'config_file': '/home/khadas/roaf3d_ws_loop/src/aruco_marker/aruco_pose/config/config.yaml'},
            {'detect_hz': 10},
            {'show_pic': False},
        ],
    )


    return LaunchDescription([
        usb_cam,
        aruco_detector_mul,
    ])
