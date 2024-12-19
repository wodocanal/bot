from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.actions import LogInfo, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    new_motors = Node(
            package='driver',
            executable='new_motors',
            name='new_motors',
            output='screen')
    
    old_motors = Node(
            package='driver',
            executable='motors',
            name='old_motors',
            output='screen')
    
    old_encoders = Node(
            package='driver',
            executable='encoders',
            name='old_encoders',
            output='screen')
    
    old_regulator = Node(
            package='driver',
            executable='regulator',
            name='old_regulator',
            output='screen')

    old_cmd_vel_encoder = Node(
            package='driver',
            executable='cmd_vel_encoder',
            name='old_encoder',
            output='screen')
    
    new = [new_motors]
    old = [old_motors, old_encoders, old_regulator, old_cmd_vel_encoder]

    return LaunchDescription([        
        *new
    ])


