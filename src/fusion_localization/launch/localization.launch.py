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
        # GPS node is commented out until hardware is available
        # Node(
        #     package='gps_reader',
        #     executable='gps_node',
        #     name='gps_node',
        #     output='screen'
        # ),
        
        # Launch the IMU node with correct executable name
        Node(
            package='imu_driver',
            executable='imu_node',  
            name='imu_node',
            output='screen'
        ),
        
        # GPS to Odometry converter (currently won't receive data without GPS node)
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
