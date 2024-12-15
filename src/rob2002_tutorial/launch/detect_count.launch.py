from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    detector_node = Node(
            package='rob2002_tutorial',
            executable='detector_basic',
            output='screen',
            emulate_tty=True,
        )
    
    counter_node = Node(
            package='rob2002_tutorial',
            executable='counter_basic',
            output='screen',
            emulate_tty=True,
        )
    
    ld.add_action(detector_node)
    ld.add_action(counter_node)

    return ld
