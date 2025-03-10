from setuptools import setup
import os
from glob import glob

package_name = 'imu_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='your.email@example.com',
    description='IMU testing package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_viz_node = imu_test.imu_viz_node:main',
            'simple_imu_node = imu_test.simple_imu_node:main',
        ],
    },
)
