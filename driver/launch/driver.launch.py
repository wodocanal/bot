from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.actions import LogInfo, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_description = get_package_share_directory('description')
    pkg_slam = get_package_share_directory('slam')
    urdf_path = os.path.join(pkg_description, 'urdf', 'robot.urdf')
    slam_main = os.path.join(pkg_slam, 'launch', 'slam.launch.py')

    odometry = Node(
            package='odometry',
            executable='odom_node',
            name='odometry',
            output='screen')
    
    motors = Node(
            package='driver',
            executable='motor_controller_node',
            name='motors',
            output='screen',)
    
    encoders = Node(
            package='driver',
            executable='encoder_node',
            name='encoders',
            output='screen')
    
    cmd_wel_enc = Node(
            package='driver',
            executable='diff_drive_controller_node',
            name='cmd_vel_enc',
            output='screen')
    
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}])
    
    joint_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_publisher',
            output='screen')
    
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_main))

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    return LaunchDescription([
        DeclareLaunchArgument('channel_type', default_value=channel_type),
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('inverted', default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='lidar',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),

        slam,
        odometry,
        
        motors,
        encoders,
        cmd_wel_enc,

        robot_state_publisher,
        joint_publisher,
    ])


