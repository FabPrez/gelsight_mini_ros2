from setuptools import setup
import os
from glob import glob

package_name = 'gsmini_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register the package in the ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='IGiuseppe Fabio Preziosa',
    maintainer_email='giuseppefabio.preziosa@polimi.it',
    description='wrapper ros2 for gelsight mini',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gsmini_img_pub = gsmini_ros2.gsmini_img_pub:main',
        ],
    },
)
