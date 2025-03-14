from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for GPS diagnostic testing."""
    return LaunchDescription([
        # Use GPS simulator with figure-eight path
        Node(
            package='fusion_localization',
            executable='gps_simulator_node.py',
            name='gps_sim',
            output='screen',
            parameters=[
                {'center_lat': 59.9127},
                {'center_lon': 10.7461},
                {'radius': 20.0},          # Larger radius for better visibility
                {'speed': 0.5},            # Slower speed for clearer pattern
                {'freq': 0.5},             # Update frequency in Hz
                {'noise_level': 0.05},     # Minimal noise for clear pattern
                {'path_type': 'figure_eight'},
            ]
        ),
        
        # GPS Diagnostic Node
        Node(
            package='fusion_localization',
            executable='gps_diagnostic_node.py',
            name='gps_diagnostic',
            output='screen',
            parameters=[
                {'max_points': 200},
            ]
        ),
        
        # Add Foxglove Bridge for remote visualization
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[
                {'port': 8765},
                {'address': '0.0.0.0'},
                {'tls': False},
                {'topic_whitelist': ['.*']},
            ]
        )
    ])
