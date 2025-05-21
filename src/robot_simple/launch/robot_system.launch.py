from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_simple',
            executable='robot_system',
            name='robot_system',
            output='screen',
        ),
    ])
