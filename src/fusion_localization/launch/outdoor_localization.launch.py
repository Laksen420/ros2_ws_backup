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
    
    # Launch arguments for configurable parameters
    use_rtk = LaunchConfiguration('use_rtk')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rtk',
            default_value='true',
            description='Whether to use RTK corrections (true) or standard GPS (false)'
        ),
        
        # Real GPS Node - note the executable name is 'gps_node' (no .py extension)
        Node(
            package='gps_reader',
            executable='gps_node',  # This matches the installed executable name
            name='gps_node',
            output='screen'
        ),
        
        # Launch the IMU node
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        
        # GPS to Odometry converter - matches the installed name with .py extension
        Node(
            package='fusion_localization',
            executable='gps_to_odom_node.py',
            name='gps_to_odom',
            output='screen',
            parameters=[
                {'use_utm': True},
                # For Denmark (Kongens Lyngby), set the appropriate UTM zone (32)
                {'utm_zone': 32},
                # Let the first GPS reading set the datum automatically
                {'datum_lat': 0.0},
                {'datum_lon': 0.0},
                # Set appropriate covariance based on RTK vs GPS modes
                {'position_covariance_rtk': 0.0025},  # 5cm accuracy squared
                {'position_covariance_gps': 5.0},     # ~2.2m accuracy squared
                {'use_rtk_covariance': use_rtk}
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
                # Denmark has a magnetic declination of approximately 3.4 degrees East (0.059 radians)
                {'magnetic_declination_radians': 0.059}, 
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
