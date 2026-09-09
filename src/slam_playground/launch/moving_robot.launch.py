from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_playground',
            executable='robot_motion',
            output='screen',
        ),

        Node(
            package='slam_playground',
            executable='laser_simulator',
            output='screen',
        ),

        Node(
            package='slam_playground',
            executable='simple_mapper',
            output='screen',
        ),
    ])
