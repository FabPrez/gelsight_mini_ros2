from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gsmini_ros2',
            executable='gsmini_img_pub',
            name='gsmini_img_pub',
            output='screen',
            # parameters=[{'some_param': 'value'}],  # se vuoi parametri
            # remappings=[('py_template_topic', 'altro_topic')],  # se vuoi remapping
        )
    ])
