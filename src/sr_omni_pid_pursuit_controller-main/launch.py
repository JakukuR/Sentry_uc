import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='pid_node',
            description='Name of the node'
        ),
        Node(
            package='pid_node',
            executable='pid_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[
                {'kp': 0.5},
                {'ki': 0.01},
                {'kd': 0.1},
                {'max': 1.0},
                {'min': -1.0},
                {'dt': 0.1}
            ]
        )
    ])