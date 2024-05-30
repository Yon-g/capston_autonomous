from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone',
            executable='local_1',
            name='local_1',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='local_2',
            name='local_2',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='local_3',
            name='local_3',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='local_4',
            name='local_4',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='map_server',
            name='map_server',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Total_Plot.py',
            name='Total_Plot',
            output='screen'
        )
    ])
