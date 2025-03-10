from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU Node (no calibration by default)
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'world_frame': 'world'},
                {'imu_frame': 'imu_link'}
            ]
        ),
        
        # Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'port': 8765}
            ]
        )
    ])
