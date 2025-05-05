from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='signal_processing',
            executable='signal_generator',
            output='screen'
        ),
        Node(
            package='signal_processing',
            executable='process',
            output='screen'
        ),
    ])
