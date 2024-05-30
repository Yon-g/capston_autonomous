from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone',
            executable='Map_1_Plot.py',
            name='Map_1_Plot',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Map_2_Plot.py',
            name='Map_2_Plot',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Map_3_Plot.py',
            name='Map_3_Plot',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Map_4_Plot.py',
            name='Map_4_Plot',
            output='screen'
        )
    ])
