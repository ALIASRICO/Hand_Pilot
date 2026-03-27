from setuptools import find_packages, setup

package_name = 'hand_tracking_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udc',
    maintainer_email='udc@todo.todo',
    description='Bridge: Meta Quest Hand Tracking to ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hand_tracking_node = hand_tracking_bridge.hand_tracking_node:main',
        ],
    },
)
