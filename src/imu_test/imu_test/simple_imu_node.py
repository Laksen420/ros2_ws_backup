#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class SimpleIMUNode(Node):
    def __init__(self):
        super().__init__('simple_imu_node')
        
        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.get_logger().info('Initializing BNO085 IMU...')
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        
        try:
            self.bno = BNO08X_I2C(self.i2c)
            self.get_logger().info('BNO085 found!')
            
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            # Create timer to publish data
            self.timer = self.create_timer(0.05, self.publish_imu_data)
            
            self.get_logger().info('Simple IMU node started')
        except Exception as e:
            self.get_logger().error(f'Error initializing BNO085: {e}')
            rclpy.shutdown()
    
    def publish_imu_data(self):
        try:
            # Get IMU data
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Fill orientation
            imu_msg.orientation.w = quat_real
            imu_msg.orientation.x = quat_i
            imu_msg.orientation.y = quat_j
            imu_msg.orientation.z = quat_k
            
            # Fill angular velocity
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            # Fill acceleration
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # Fill covariance with placeholder values
            imu_msg.orientation_covariance = [0.01] * 9
            imu_msg.angular_velocity_covariance = [0.01] * 9
            imu_msg.linear_acceleration_covariance = [0.01] * 9
            
            # Publish data
            self.imu_pub.publish(imu_msg)
            
            # Print for debugging
            self.get_logger().info(
                f'Publishing IMU data:\n'
                f'  Quaternion: [{quat_real:.2f}, {quat_i:.2f}, {quat_j:.2f}, {quat_k:.2f}]\n'
                f'  Angular velocity: [{gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}]\n'
                f'  Acceleration: [{accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}]'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error reading from BNO085: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down simple IMU node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
