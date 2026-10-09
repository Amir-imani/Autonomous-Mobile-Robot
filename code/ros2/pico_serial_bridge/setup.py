import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pico_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amir Imanipanah',
    maintainer_email='amir.imanipanah@gmail.com',
    description='Robust binary serial bridge for a Raspberry Pi Pico.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'differential_odometry_node = pico_serial_bridge.differential_odometry_node:main',
        'lidar_scan_node = pico_serial_bridge.lidar_scan_node:main',
        'serial_bridge_node = pico_serial_bridge.serial_bridge_node:main',
    ]},
)
