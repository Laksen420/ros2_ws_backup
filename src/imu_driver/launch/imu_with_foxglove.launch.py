import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('forklift_description'),
        'urdf',
        'forklift_description.urdf'
    )
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Static transform publisher for world frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )
    
    # IMU Node
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'world_frame': 'world',
            'imu1_frame': 'imu1_link',
            'imu2_frame': 'imu2_link'
        }]
    )
    
    # Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': "172.26.124.22",
            'send_buffer_limit': 10000000,
            'topic_whitelist': [".*"],
            'service_whitelist': [".*"]
        }]
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        static_transform_publisher,
        imu_node,
        foxglove_bridge
    ])
