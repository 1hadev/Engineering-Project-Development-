from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='nav2_bringup'
        ),
        Node(
            package='my_jetrover_nav',
            executable='text_nav.py',
            name='text_navigator'
        )
    ])
