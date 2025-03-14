from setuptools import setup
import os
from glob import glob
package_name = 'fusion_localization'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include rviz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='your-email@example.com',
    description='Sensor fusion for RTK GPS and IMU using robot_localization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_to_odom_node.py = fusion_localization.gps_to_odom_node:main',
            # Remove the incorrect reference to gps_node that belongs to another package
            # 'gps_node.py = fusion_localization.gps_node:main',  
            # Uncomment if you want to use the simulator again
            # 'gps_simulator_node.py = fusion_localization.gps_simulator_node:main',
        ],
    },
)
