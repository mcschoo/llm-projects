from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gemini_ros2_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include any future launch files we decide to implement
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'google-generativeai', # Add Gemini library dependency
        'rclpy',             # Already added via pkg create, here for redundancy
        'std_msgs',          # Already added via pkg create, here for redundancy
    ],
    zip_safe=True,
    maintainer='user', # universal placeholder
    maintainer_email='user@todo.todo', # universal placeholder
    description='ROS 2 nodes for interfacing with Gemini API for commands.', 
    license='Apache License 2.0', # any liscence works
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add entry points for both nodes
            'command_input_node = gemini_ros2_interface.command_input_node:main',
            'gemini_processor_node = gemini_ros2_interface.gemini_processor_node:main',
        ],
    },
)