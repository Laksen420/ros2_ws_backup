#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import time

class IMUVizNode(Node):
    def __init__(self):
        super().__init__('imu_viz_node')
        
        # Create publishers for IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Set up transform broadcaster for visualization
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Initializing BNO085 IMU...')
        
        # Create I2C bus with 100kHz clock (as you mentioned was working)
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        
        try:
            self.bno = BNO08X_I2C(self.i2c)
            self.get_logger().info('BNO085 found!')
            
            # Enable necessary features
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            
            # Create a timer to publish IMU data at 20Hz
            self.timer = self.create_timer(0.05, self.publish_imu_data)
            
            self.get_logger().info('IMU visualization node started')
        except Exception as e:
            self.get_logger().error(f'Error initializing BNO085: {e}')
            rclpy.shutdown()
    
    def publish_imu_data(self):
        try:
            # Get timestamp for messages
            now = self.get_clock().now().to_msg()
            
            # Get data from IMU
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            
            # Create and publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'imu_link'
            
            # Set orientation (from rotation vector)
            imu_msg.orientation.w = quat_real
            imu_msg.orientation.x = quat_i
            imu_msg.orientation.y = quat_j
            imu_msg.orientation.z = quat_k
            
            # Set angular velocity
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # Set covariance (placeholder values)
            # In a real application, you'd want to tune these based on sensor characteristics
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Publish IMU data
            self.imu_pub.publish(imu_msg)
            
            # Broadcast transform for visualization
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = 'imu_link'
            
            # Set translation (fixed position for visualization)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # Set rotation from quaternion
            t.transform.rotation.w = quat_real
            t.transform.rotation.x = quat_i
            t.transform.rotation.y = quat_j
            t.transform.rotation.z = quat_k
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error reading from BNO085: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUVizNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down IMU visualization node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
