from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gemini_ros2_interface'

setup(
    name=package_name,
    version='0.0.3', # Increment version maybe
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'google-generativeai',
        'rclpy',
        'std_msgs',
        'transformers',
        'torch', 
        # Add torchvision/torchaudio if needed --- we will need torchaudio later
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='ROS 2 nodes for Gemini API with summarization pre-processing.', # Updated desc
    license='Apache License 2.0', # can swap to MIT if we care
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Entry point for the original command input node
            'command_input_node = gemini_ros2_interface.command_input_node:main',
            # Entry point for the summarizer processing node
            'summarizer_processor_node = gemini_ros2_interface.text_summarizer:main', # Assumes filename is text_summarizer.py
            # Entry point for the Gemini processing node
            'gemini_processor_node = gemini_ros2_interface.gemini_processor_node:main',
        ],
    },
)