from setuptools import setup

package_name = 'pallet_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your_email@example.com',
    description='Nodes for triggering and fusing pallet events',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_node = pallet_nodes.trigger_node:main',
            'pallet_fusion_node = pallet_nodes.pallet_fusion_node:main'
        ],
    },
)
