import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vicon_host_arg = DeclareLaunchArgument(
        'vicon_host',
        default_value='localhost:801',
        description='Vicon DataStream host and port'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='vicon',
        description='Namespace for the vicon odom publisher'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)'
    )
    
    vicon_odom_node = Node(
        package='auna_vicon',
        executable='odom_publisher_main',
        name='odom_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'vicon_host': LaunchConfiguration('vicon_host'),
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        vicon_host_arg,
        namespace_arg, 
        log_level_arg,
        vicon_odom_node
    ])
