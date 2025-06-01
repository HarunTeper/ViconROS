from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auna_vicon',
            executable='gps_publisher_main',
            name='gps_publisher',
            output='screen',
        )
    ])
