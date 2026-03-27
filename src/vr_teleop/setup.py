import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vr_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udc',
    maintainer_email='udc@todo.todo',
    description='VR hand tracking teleoperation for Dobot CR20',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = vr_teleop.teleop_node:main',
            'calibrate_axes = vr_teleop.calibrate_axes:main',
        ],
    },
)
