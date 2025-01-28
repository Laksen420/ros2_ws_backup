from setuptools import setup
import os
from glob import glob

package_name = 'gps_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynmea2', 'paho-mqtt'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your_email@example.com',
    description='GPS reading node for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_reader.gps_node:main'
        ],
    },
)
