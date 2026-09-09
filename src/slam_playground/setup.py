from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'slam_playground'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='moritz.a.weber02@gmail.com',
    description='Simple SLAM-Implementation for learning purposes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_motion = slam_playground.moving_robot.robot_motion_node:main',
            'laser_simulator = slam_playground.moving_robot.laser_simulator_node:main',
            'simple_mapper = slam_playground.moving_robot.simple_mapper_node:main',
        ],
    },
)
