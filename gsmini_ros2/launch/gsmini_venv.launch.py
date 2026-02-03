from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    python_bin = "/home/fabioprez/projects/venv_projects/gsmini_venv/bin/python3"
    device_id = LaunchConfiguration('device_id')
    topic = LaunchConfiguration('topic')
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='1',
            description='GelSight camera device id'
        ),
        
        DeclareLaunchArgument(
            'topic',
            default_value='/dito_1/image',
        ),
        

        Node(
            package='gsmini_ros2',
            executable='gsmini_img_pub',
            name='gsmini_img_pub',
            output='screen',
            arguments=[
                '--device-id', device_id,
                '--topic', topic
            ],
            remappings=[('/camer', 'altro_topic')],  # se vuoi remapping
            prefix=python_bin
        )
    ])
