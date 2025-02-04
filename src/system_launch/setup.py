from setuptools import setup
import os
from glob import glob

package_name = 'system_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files into share/system_launch/launch
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your_email@example.com',
    description='Launch file package to start all nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # No console scripts needed for a pure launch package.
    },
)
