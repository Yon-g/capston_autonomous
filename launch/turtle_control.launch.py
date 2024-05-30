from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone',
            executable='Turtle_1_Control.py',
            name='Turtle_1_Control',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Turtle_2_Control.py',
            name='Turtle_2_Control',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Turtle_3_Control.py',
            name='Turtle_3_Control',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='Turtle_4_Control.py',
            name='Turtle_4_Control',
            output='screen'
        )
    ])
