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
   
    aruco_tracking = Node(
        package='aruco_tracking',
        executable='aruco_tarcking_node',
        parameters=[
            { 
                'P' : 1.5,
                'I' : 0.0, 
                'D' : 0.0,
                'Sum_err' : 0.05,           #积分项限幅 
                'Fall_speed' : 0.20,        #下降速度 m/s
                'yaw_p' : 1.0,              #角速度P rad/s
                'camera_time' : 5.0,        #拍照等待时长 s
                'over_time_s' : 5.5,        #二维码识别超时阈值 s
                'max_vel_abs' : 0.6,        #单轴最大速度 m/s
                'norm_fly_height' : 1.0,    #起飞高度 m
                'set_point_x' : 10.0,       #设定的x坐标 m
                'set_point_y' : 0.0,        #设定的y坐标 m
                'tracking_mode' : 1.0,      #循迹模式 0.0 定点 1.0 定速
                'tracking_radius' : 0.15,   #循迹跑点的判定半径
                'tracking_angle' : 45.0,    #循迹的角度限定 
            },
        ],

    )

    return LaunchDescription([
        aruco_tracking,
    ])
