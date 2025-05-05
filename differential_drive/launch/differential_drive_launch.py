from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo para publicar el modelo del robot desde el archivo URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/felipe/ros2_ws/src/my_robot.urdf').read()}]
        ),

        # Nodo para abrir RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Nodo para differential_drive_node
        Node(
            package='differential_drive',
            executable='differential_drive_node',
            name='differential_drive_node',
            output='screen'
        ),
        Node(
            package='differential_drive',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen'
        )
    ])
