import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():
    # Declare launch arguments
    vicon_host_arg = DeclareLaunchArgument(
        'vicon_host',
        default_value='localhost:801',
        description='Vicon DataStream host and port (e.g., 192.168.1.100:801)'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='vicon',
        description='Namespace for the vicon marker publisher'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)'
    )
    
    # High-performance Vicon marker publisher node
    vicon_marker_node = Node(
        package='auna_vicon',  # Replace with your package name
        executable='marker_publisher_main',
        name='marker_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'vicon_host': LaunchConfiguration('vicon_host'),
            'use_sim_time': False,  # Use real-time
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        vicon_host_arg,
        namespace_arg, 
        log_level_arg,
        vicon_marker_node
    ])