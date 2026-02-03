#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Absolute path to your venv Python interpreter
    python_bin = "/home/fabioprez/projects/venv_projects/gsmini_venv/bin/python3"

    return LaunchDescription([
        Node(
            package='gsmini_ros2',
            executable='gsmini_img_pub',
            name='gsmini_img_pub',
            output='screen',
            prefix=python_bin
        )
    ])
