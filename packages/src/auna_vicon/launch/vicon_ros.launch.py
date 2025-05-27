from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auna_vicon',
            executable='vicon_ros',
            name='vicon_ros',
            output='screen'
        )
    ])