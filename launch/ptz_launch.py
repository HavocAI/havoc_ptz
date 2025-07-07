from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='havoc_ptz',
            executable='ptz_node',
            name='ptz_node',
            output='screen',
            parameters=[]
        ),
    ])
