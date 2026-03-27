from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dobot_camera'

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
    description='Cámara RealSense con AprilTags y YOLO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = dobot_camera.vision_node:main',
            'yolo_detector = dobot_camera.yolo_detector:main',
        ],
    },
)
