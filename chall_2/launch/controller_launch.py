import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('chall_2'), 'config', 'controller_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='chall_2',
            executable='ctrl',
            name='ctrl',
            parameters=[config_file]
        )
    ])
