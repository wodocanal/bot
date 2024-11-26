from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['slam/config/slam_toolbox_params.yaml']
        ),
        Node(
            package='slam',
            executable='slam_manual_mapping',
            name='manual_mapping',
            output='screen',
        )
    ])
