import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    navigation_pkg_share = FindPackageShare('navigation').find('navigation')

    return LaunchDescription([
        # Include the main navigation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_pkg_share, 'launch', 'navigation.launch.py')
            )
        ),

        # Include the RViz visualization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_pkg_share, 'launch', 'rviz_navigation.launch.py')
            )
        ),

                   # Launch YOUR text_nav node
        Node(
            package='text_nav',
            executable='text_nav_node',
            name='text_nav',
            output='screen'
        ),
    ])
