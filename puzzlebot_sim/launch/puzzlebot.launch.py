from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():
    urdf_path= os.path.join(
          os.getenv('HOME'), 'ros2_ws', 'src', 'puzzlebot_sim', 'urdf', 'puzzlebot.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description' : open(urdf_path).read()}],
            output='screen'
        ),
        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_simulator',
            name='puzzlebot_simulator',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[],
            output='screen'
        )


    ])