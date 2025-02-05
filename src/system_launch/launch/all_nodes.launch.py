from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rfid_ros2',
            executable='rfid_node',
            name='rfid_node'
        ),
        Node(
            package='lidar_trigger',
            executable='lidar_trigger_node',
            name='lidar_trigger_node'
        ),
        Node(
            package='gps_reader',
            executable='gps_node',
            name='gps_node'
        ),
        Node(
            package='azure_sender',
            executable='azure_node',
            name='azure_node'
        ),
    ])

