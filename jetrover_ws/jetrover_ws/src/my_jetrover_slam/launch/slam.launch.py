from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

     Node(
    package='rplidar_ros',
    executable='rplidar_node',
    name='rplidar_node',
    parameters=[{
        'serial_port': '/dev/ttyUSB0',
        'frame_id': 'laser',
        'angle_compensate': True,
        'scan_mode': 'Standard'
    }]
         
),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='my_slam_node',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='my_rviz2',
            arguments=['-d', '/home/ubuntu/jetrover_ws/src/my_jetrover_slam/rviz/slam_config.rviz']
        )
    ])
    
