from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auna_vicon',
            executable='marker_publisher_main',
            name='marker_publisher_main',
            output='screen'
        )
    ])
