from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting IMU calibration sequence..."),
        
        LogInfo(msg="CALIBRATION INSTRUCTIONS:"),
        LogInfo(msg="1. Keep the platform still for 10 seconds"),
        LogInfo(msg="2. Slowly tilt/rotate the platform in different directions"),
        LogInfo(msg="3. If possible, drive in a figure-8 pattern"),
        
        Node(
            package='imu_driver',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'enable_startup_calibration': True},  # Explicitly enable calibration
                {'world_frame': 'world'},
                {'imu_frame': 'imu_link'}
            ]
        )
    ])
