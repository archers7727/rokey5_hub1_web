from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_hub_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'supabase',
        'python-dotenv',
    ],
    zip_safe=True,
    maintainer='Robot Hub Team',
    maintainer_email='dev@example.com',
    description='ROS2 nodes for robot hub - Supabase integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = robot_hub_nodes.robot_state_publisher_node:main',
            'task_monitor = robot_hub_nodes.task_monitor_node:main',
            'test_task_subscriber = robot_hub_nodes.test_task_subscriber:main',
        ],
    },
)
