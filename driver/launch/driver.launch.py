from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='driver',
            executable='motor_controller',
            name='motor_controller_node',
            output='screen',
        ),
        Node(
            package='driver',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
        ),
    ])
