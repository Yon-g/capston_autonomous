from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone',
            executable='planning_1',
            name='planning_1',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='planning_2',
            name='planning_2',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='planning_3',
            name='planning_3',
            output='screen'
        ),
        Node(
            package='capstone',
            executable='planning_4',
            name='planning_4',
            output='screen'
        )
    ])
