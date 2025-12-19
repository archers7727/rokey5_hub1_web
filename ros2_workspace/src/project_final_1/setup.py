from setuptools import find_packages, setup

package_name = 'project_final_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='archer',
    maintainer_email='archering7727@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_handler = project_final_1.robot_handler:main',
            'robot_state_publisher = project_final_1.robot_state_publisher:main',
        ],
    },
)
