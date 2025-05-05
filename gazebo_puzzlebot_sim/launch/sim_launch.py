from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gazebo_puzzlebot_sim'),
        'config',
        'path.yaml'
    )

    return LaunchDescription([
        Node(
            package='gazebo_puzzlebot_sim',
            executable='sim_node',
            name='sim_node',
            output='screen',
            parameters=[{
                'path_file': config_file
            }]
        )
    ])
