from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Minimal publisher test node
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_publisher',
            output='screen'
        ),
        
        # Foxglove Bridge with minimal configuration
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'port': 8765,
                 'address': "172.26.124.22",  # Your ZeroTier IP
                 'send_buffer_limit': 1024000,  # Smaller buffer
                 'retry_wait_ms': 1000,  # More aggressive retry
                 'heartbeat_interval_ms': 1000  # More frequent heartbeats
                }
            ]
        )
    ])
