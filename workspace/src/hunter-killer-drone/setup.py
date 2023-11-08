#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'hunter_killer_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', glob('resource/*')),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TelevisionNinja',
    maintainer_email='televisionninja@gmail.com',
    description='Firmware and simulation of a hunter killer drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualizer = hunter_killer_drone.visualizer:main',
                'offboard_control = hunter_killer_drone.offboard_control:main',
                'control = hunter_killer_drone.control:main',
                'processes = hunter_killer_drone.processes:main'
        ],
    },
)
