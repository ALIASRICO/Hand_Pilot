from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dobot_moveit'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IUDC',
    maintainer_email='iudc@example.com',
    description='MoveIt integration for Dobot CR20',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_states = dobot_moveit.joint_states:main',
            'action_move_server = dobot_moveit.action_move_server:main',
            'vision_coordinator = dobot_moveit.vision_coordinator:main',
            'calibration_node = dobot_moveit.calibration_node:main',
            'depalletizer = dobot_moveit.depalletizer:main',
            'scene_manager = dobot_moveit.scene_manager:main',
            'collision_calibrator = dobot_moveit.collision_calibrator:main',
        ],
    },
)
