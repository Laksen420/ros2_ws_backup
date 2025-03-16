from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('forklift_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'forklift_description.urdf')
    
    # Load URDF
    robot_description = Command([FindExecutable(name='cat'), ' ', urdf_file])
    
    # Publish joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
