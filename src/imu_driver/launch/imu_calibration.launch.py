from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction

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
                {'perform_calibration': True},
                {'calibration_timeout': 60.0}  # Longer timeout for thorough calibration
            ]
        ),
        
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])
