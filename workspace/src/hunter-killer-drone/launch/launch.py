#!/usr/bin/env python


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('hunter_killer_drone')
    return LaunchDescription([
        Node(
            package='hunter_killer_drone',
            namespace='hunter_killer_drone',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='hunter_killer_drone',
            namespace='hunter_killer_drone',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),
        Node(
            package='hunter_killer_drone',
            namespace='hunter_killer_drone',
            executable='offboard_control',
            name='offboard_control'
        ),
        Node(
            package='hunter_killer_drone',
            namespace='hunter_killer_drone',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='hunter_killer_drone',
            namespace='',
            executable='vision',
            name='vision'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
