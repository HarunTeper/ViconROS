from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vicon_ros',
            executable='vicon_ros',
            name='vicon_ros',
            output='screen'
        )
    ])