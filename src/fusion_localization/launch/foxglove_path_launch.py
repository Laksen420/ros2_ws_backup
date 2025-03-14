from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the path to your config file
    ekf_config_file = os.path.join(
        get_package_share_directory('fusion_localization'),
        'config',
        'ekf_config.yaml'
    )
    
    return LaunchDescription([
        # Use GPS simulator instead of real GPS node
        Node(
            package='fusion_localization',
            executable='gps_simulator_node.py',
            name='gps_sim',
            output='screen',
            parameters=[
                {'center_lat': 59.9127},  # Starting position
                {'center_lon': 10.7461},
                {'radius': 10.0},         # 10 meter circle
                {'speed': 1.0},           # 1 m/s
                {'freq': 1.0},            # 1 Hz update rate
            ]
        ),
        
        # Launch the IMU node
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        
        # GPS to Odometry converter
        Node(
            package='fusion_localization',
            executable='gps_to_odom_node.py',
            name='gps_to_odom',
            output='screen',
            parameters=[
                {'use_utm': True},
                {'datum_lat': 0.0},
                {'datum_lon': 0.0},
            ]
        ),
        
        # Launch robot_localization EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file]
        ),
        
        # NavSat transform node to handle GPS conversions
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'frequency': 30.0},
                {'delay': 0.0},
                {'magnetic_declination_radians': 0.0},
                {'yaw_offset': 0.0},
                {'zero_altitude': False},
                {'publish_filtered_gps': True},
                {'use_odometry_yaw': False},
                {'wait_for_datum': False},
            ],
            remappings=[
                ('imu', 'imu/data'),
                ('gps/fix', 'gps_data'),
                ('odometry/filtered', 'odometry/filtered'),
            ]
        ),
        
        # Foxglove-friendly Path Publisher Node
        Node(
            package='fusion_localization',
            executable='foxglove_path_publisher_node.py',
            name='foxglove_path_publisher',
            output='screen',
            parameters=[
                {'max_points': 1000},
                {'save_path': True},
                {'save_interval': 5.0},
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
