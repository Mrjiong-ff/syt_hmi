from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='syt_hmi',
            executable='syt_hmi',
            name='syt_hmi_node'),
    ])
