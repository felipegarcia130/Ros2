from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    generator_node = Node(name="signal_generator",
                       package='signal_proc',
                       executable='signal_gen',
                       output='screen'
                       )
    
    process_node = Node(name="signal_process",
                         package='signal_proc',
                         executable='signal_proc',
                         output='screen'
                         )
    
    rqt_node = Node(name='rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/signal/data', '/proc_signal/data']
                    )
    
    l_d = LaunchDescription([generator_node, process_node, rqt_node])

    return l_d