#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Bool
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
import math

class MultiIMUNode(Node):
   def __init__(self):
       super().__init__('multi_imu_node')
       
       # Declare parameters
       self.declare_parameter('enable_startup_calibration', False)
       self.declare_parameter('world_frame', 'world')
       self.declare_parameter('imu1_frame', 'imu1_link')
       self.declare_parameter('imu2_frame', 'imu2_link')

       # Get parameter values
       self.in_startup_calibration = self.get_parameter('enable_startup_calibration').value
       self.world_frame = self.get_parameter('world_frame').value
       self.imu1_frame = self.get_parameter('imu1_frame').value
       self.imu2_frame = self.get_parameter('imu2_frame').value
       
       # Create publishers for IMU 1
       self.imu1_pub = self.create_publisher(Imu, 'imu1/data', 10)
       self.calib1_pub = self.create_publisher(String, 'imu1/calibration_status', 10)
       self.calib1_complete_pub = self.create_publisher(Bool, 'imu1/calibration_complete', 10)
       
       # Create publishers for IMU 2
       self.imu2_pub = self.create_publisher(Imu, 'imu2/data', 10)
       self.calib2_pub = self.create_publisher(String, 'imu2/calibration_status', 10)
       self.calib2_complete_pub = self.create_publisher(Bool, 'imu2/calibration_complete', 10)
       
       # Create a combined publisher
       self.combined_pub = self.create_publisher(Imu, 'imu/combined_data', 10)

       # Create a TF broadcaster
       self.tf_broadcaster = TransformBroadcaster(self)

       # Calibration parameters for IMU 1
       self.imu1_gyro_samples = []
       self.imu1_accel_samples = []
       self.imu1_mag_samples = []
       self.imu1_last_accel = None
       self.imu1_frozen_counter = 0
       self.imu1_error_counter = 0
       
       # Calibration parameters for IMU 2
       self.imu2_gyro_samples = []
       self.imu2_accel_samples = []
       self.imu2_mag_samples = []
       self.imu2_last_accel = None
       self.imu2_frozen_counter = 0
       self.imu2_error_counter = 0
       
       # Shared calibration parameters
       self.calibration_timeout = 30.0  # 30 seconds for initial calibration
       self.calibration_start_time = None
       self.in_startup_calibration = True  # Set to True for initial calibration

       # Initialize I2C
       self.i2c = busio.I2C(board.SCL, board.SDA)
       time.sleep(0.5)  # Wait for I2C to initialize
       
       # Initialize both IMUs
       self.bno1 = None  # Will be set in init_imu1
       self.bno2 = None  # Will be set in init_imu2
       imu1_ok = self.init_imu1()
       imu2_ok = self.init_imu2()
       
       if not (imu1_ok and imu2_ok):
           self.get_logger().error("Failed to initialize one or both IMUs!")
       else:
           self.get_logger().info("Both IMUs initialized successfully!")

       # Create timers
       self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10Hz IMU data
       self.calib_timer = self.create_timer(1.0, self.publish_calibration_status)  # 1Hz calibration status

       # Start calibration
       self.calibration_start_time = self.get_clock().now().seconds_nanoseconds()[0]
       self.get_logger().info('Multi-IMU Node started - beginning calibration sequence')
       self.get_logger().info('For best results during calibration:')
       self.get_logger().info('1. Keep the platform still for the first 10 seconds')
       self.get_logger().info('2. Then slowly rotate/tilt the platform in different directions')
       self.get_logger().info('3. Finally, drive in a figure-8 pattern if possible')

   def init_imu1(self):
       """Initialize the first IMU"""
       try:
           self.get_logger().info('Initializing first BNO085 IMU at address 0x4A...')
           # Primary IMU at default 0x4A address
           self.bno1 = BNO08X_I2C(self.i2c)  # Default address is 0x4A

           # Enable features
           self.bno1.enable_feature(BNO_REPORT_ACCELEROMETER)
           self.bno1.enable_feature(BNO_REPORT_GYROSCOPE)
           self.bno1.enable_feature(BNO_REPORT_MAGNETOMETER)
           self.bno1.enable_feature(BNO_REPORT_ROTATION_VECTOR)

           # Reset counters
           self.imu1_frozen_counter = 0
           self.imu1_error_counter = 0

           # Reset calibration data
           self.imu1_gyro_samples = []
           self.imu1_accel_samples = []
           self.imu1_mag_samples = []

           self.get_logger().info('First BNO085 initialized successfully!')
           return True

       except Exception as e:
           self.get_logger().error(f'Failed to initialize first IMU: {e}')
           return False
           
   def init_imu2(self):
       """Initialize the second IMU"""
       try:
           self.get_logger().info('Initializing second BNO085 IMU at address 0x4B...')
           
           # Second IMU with address 0x4B (DI pin is pulled high)
           self.bno2 = BNO08X_I2C(self.i2c, address=0x4B)

           # Enable features
           self.bno2.enable_feature(BNO_REPORT_ACCELEROMETER)
           self.bno2.enable_feature(BNO_REPORT_GYROSCOPE)
           self.bno2.enable_feature(BNO_REPORT_MAGNETOMETER)
           self.bno2.enable_feature(BNO_REPORT_ROTATION_VECTOR)

           # Reset counters
           self.imu2_frozen_counter = 0
           self.imu2_error_counter = 0

           # Reset calibration data
           self.imu2_gyro_samples = []
           self.imu2_accel_samples = []
           self.imu2_mag_samples = []

           self.get_logger().info('Second BNO085 initialized successfully!')
           return True

       except Exception as e:
           self.get_logger().error(f'Failed to initialize second IMU: {e}')
           return False

   def reset_imu1_if_needed(self):
       """Reset the first IMU if too many errors or frozen data"""
       self.get_logger().warn('Resetting first IMU...')
       try:
           del self.bno1
       except:
           pass

       time.sleep(1.0)  # Wait before reinitializing
       return self.init_imu1()
       
   def reset_imu2_if_needed(self):
       """Reset the second IMU if too many errors or frozen data"""
       self.get_logger().warn('Resetting second IMU...')
       try:
           del self.bno2
       except:
           pass

       time.sleep(1.0)  # Wait before reinitializing
       return self.init_imu2()

   def evaluate_calibration(self, gyro_samples, accel_samples, mag_samples, imu_name="IMU"):
       """Evaluate the calibration quality based on collected samples"""
       if not gyro_samples or not accel_samples or not mag_samples:
           return False, f"Insufficient data for {imu_name} calibration evaluation"

       # Check gyro stability (should be close to zero when stationary)
       gyro_avg = [sum(axis) / len(axis) for axis in zip(*gyro_samples)]
       gyro_std = [
           math.sqrt(sum((val - gyro_avg[i])**2 for val in axis) / len(axis))
           for i, axis in enumerate(zip(*gyro_samples))
       ]
       gyro_ok = all(std < 0.05 for std in gyro_std)

       # Check accelerometer (magnitude should be close to g = 9.81 m/s²)
       accel_magnitudes = [
           math.sqrt(x**2 + y**2 + z**2)
           for x, y, z in accel_samples
       ]
       accel_avg_mag = sum(accel_magnitudes) / len(accel_magnitudes)
       accel_ok = abs(accel_avg_mag - 9.81) < 0.5

       # Check magnetometer (should have reasonable variation for calibration)
       mag_min = [min(axis) for axis in zip(*mag_samples)]
       mag_max = [max(axis) for axis in zip(*mag_samples)]
       mag_range = [max_val - min_val for min_val, max_val in zip(mag_min, mag_max)]
       mag_ok = all(rng > 5.0 for rng in mag_range)  # Should have at least 5μT range in each axis

       # Overall calibration status
       calib_ok = gyro_ok and accel_ok and mag_ok

       status_msg = (
           f"{imu_name} Calibration:\n"
           f"Gyroscope: {'OK' if gyro_ok else 'Needs calibration'} (std: {gyro_std})\n"
           f"Accelerometer: {'OK' if accel_ok else 'Needs calibration'} (avg mag: {accel_avg_mag:.2f})\n"
           f"Magnetometer: {'OK' if mag_ok else 'Needs calibration'} (range: {mag_range})"
       )

       return calib_ok, status_msg

   def process_startup_calibration(self):
       """Process the startup calibration sequence for both IMUs"""
       current_time = self.get_clock().now().seconds_nanoseconds()[0]
       elapsed_time = current_time - self.calibration_start_time

       # Collect sensor data for calibration evaluation
       try:
           # IMU 1 calibration data collection
           try:
               gyro1 = self.bno1.gyro
               accel1 = self.bno1.acceleration
               mag1 = self.bno1.magnetic

               self.imu1_gyro_samples.append(gyro1)
               self.imu1_accel_samples.append(accel1)
               self.imu1_mag_samples.append(mag1)

               # Limit sample size
               max_samples = 100
               if len(self.imu1_gyro_samples) > max_samples:
                   self.imu1_gyro_samples.pop(0)
               if len(self.imu1_accel_samples) > max_samples:
                   self.imu1_accel_samples.pop(0)
               if len(self.imu1_mag_samples) > max_samples:
                   self.imu1_mag_samples.pop(0)
           except Exception as e:
               self.get_logger().warn(f"Error collecting IMU1 calibration data: {e}")
               
           # IMU 2 calibration data collection
           try:
               gyro2 = self.bno2.gyro
               accel2 = self.bno2.acceleration
               mag2 = self.bno2.magnetic

               self.imu2_gyro_samples.append(gyro2)
               self.imu2_accel_samples.append(accel2)
               self.imu2_mag_samples.append(mag2)

               # Limit sample size
               max_samples = 100
               if len(self.imu2_gyro_samples) > max_samples:
                   self.imu2_gyro_samples.pop(0)
               if len(self.imu2_accel_samples) > max_samples:
                   self.imu2_accel_samples.pop(0)
               if len(self.imu2_mag_samples) > max_samples:
                   self.imu2_mag_samples.pop(0)
           except Exception as e:
               self.get_logger().warn(f"Error collecting IMU2 calibration data: {e}")

           # Provide guidance based on elapsed time
           phase_msg = ""
           if elapsed_time < 10.0:
               phase_msg = "Phase 1/3: Keep platform still for gyro calibration"
           elif elapsed_time < 20.0:
               phase_msg = "Phase 2/3: Slowly tilt/rotate platform for accel calibration"
           else:
               phase_msg = "Phase 3/3: Drive in figure-8 pattern for mag calibration"

           # Check if calibration time has elapsed
           if elapsed_time >= self.calibration_timeout:
               # Evaluate IMU 1
               calib1_ok, status1 = self.evaluate_calibration(
                   self.imu1_gyro_samples, 
                   self.imu1_accel_samples, 
                   self.imu1_mag_samples,
                   "IMU1"
               )
               
               # Evaluate IMU 2
               calib2_ok, status2 = self.evaluate_calibration(
                   self.imu2_gyro_samples, 
                   self.imu2_accel_samples, 
                   self.imu2_mag_samples,
                   "IMU2"
               )

               # Publish calibration complete messages
               complete1_msg = Bool()
               complete1_msg.data = calib1_ok
               self.calib1_complete_pub.publish(complete1_msg)
               
               complete2_msg = Bool()
               complete2_msg.data = calib2_ok
               self.calib2_complete_pub.publish(complete2_msg)

               self.get_logger().info("Calibration complete!")
               self.get_logger().info(status1)
               self.get_logger().info(status2)

               self.in_startup_calibration = False
               return

           # Update calibration status for both IMUs
           calib1_msg = String()
           calib1_msg.data = (
               f"IMU1 CALIBRATION IN PROGRESS: {elapsed_time:.1f}/{self.calibration_timeout:.1f}s\n"
               f"{phase_msg}\n"
               f"Samples Collected: {len(self.imu1_gyro_samples)}"
           )
           self.calib1_pub.publish(calib1_msg)
           
           calib2_msg = String()
           calib2_msg.data = (
               f"IMU2 CALIBRATION IN PROGRESS: {elapsed_time:.1f}/{self.calibration_timeout:.1f}s\n"
               f"{phase_msg}\n"
               f"Samples Collected: {len(self.imu2_gyro_samples)}"
           )
           self.calib2_pub.publish(calib2_msg)

       except Exception as e:
           self.get_logger().error(f"Error during calibration: {e}")

   def publish_calibration_status(self):
       """Publish calibration status information for both IMUs"""
       # During startup calibration, handle differently
       if self.in_startup_calibration:
           self.process_startup_calibration()
           return

       # IMU 1 calibration status
       try:
           accel1 = self.bno1.acceleration
           gyro1 = self.bno1.gyro
           mag1 = self.bno1.magnetic

           # For gyro, check if values are close to zero when stationary
           gyro1_calibrated = all(abs(v) < 0.05 for v in gyro1)

           # For accelerometer, check if magnitude is close to 9.8 m/s²
           accel1_magnitude = (accel1[0]**2 + accel1[1]**2 + accel1[2]**2)**0.5
           accel1_calibrated = abs(accel1_magnitude - 9.81) < 0.5

           # For magnetometer, check if values are reasonable
           mag1_magnitude = (mag1[0]**2 + mag1[1]**2 + mag1[2]**2)**0.5
           mag1_calibrated = 25 < mag1_magnitude < 65  # Typical range for Earth's magnetic field

           # Create calibration message
           calib1_msg = String()
           calib1_msg.data = (
               f"IMU1 Calibration Status:\n"
               f"Accelerometer: {'Calibrated' if accel1_calibrated else 'Needs Calibration'}\n"
               f"Gyroscope: {'Calibrated' if gyro1_calibrated else 'Needs Calibration'}\n"
               f"Magnetometer: {'Calibrated' if mag1_calibrated else 'Needs Calibration'}\n"
               f"Accel magnitude: {accel1_magnitude:.2f} m/s² (should be ~9.8)\n"
               f"Mag magnitude: {mag1_magnitude:.2f} μT (should be ~25-65)"
           )

           self.calib1_pub.publish(calib1_msg)
       except Exception as e:
           self.get_logger().error(f'Error publishing IMU1 calibration status: {e}')
           
       # IMU 2 calibration status
       try:
           accel2 = self.bno2.acceleration
           gyro2 = self.bno2.gyro
           mag2 = self.bno2.magnetic

           # For gyro, check if values are close to zero when stationary
           gyro2_calibrated = all(abs(v) < 0.05 for v in gyro2)

           # For accelerometer, check if magnitude is close to 9.8 m/s²
           accel2_magnitude = (accel2[0]**2 + accel2[1]**2 + accel2[2]**2)**0.5
           accel2_calibrated = abs(accel2_magnitude - 9.81) < 0.5

           # For magnetometer, check if values are reasonable
           mag2_magnitude = (mag2[0]**2 + mag2[1]**2 + mag2[2]**2)**0.5
           mag2_calibrated = 25 < mag2_magnitude < 65  # Typical range for Earth's magnetic field

           # Create calibration message
           calib2_msg = String()
           calib2_msg.data = (
               f"IMU2 Calibration Status:\n"
               f"Accelerometer: {'Calibrated' if accel2_calibrated else 'Needs Calibration'}\n"
               f"Gyroscope: {'Calibrated' if gyro2_calibrated else 'Needs Calibration'}\n"
               f"Magnetometer: {'Calibrated' if mag2_calibrated else 'Needs Calibration'}\n"
               f"Accel magnitude: {accel2_magnitude:.2f} m/s² (should be ~9.8)\n"
               f"Mag magnitude: {mag2_magnitude:.2f} μT (should be ~25-65)"
           )

           self.calib2_pub.publish(calib2_msg)
       except Exception as e:
           self.get_logger().error(f'Error publishing IMU2 calibration status: {e}')

   def publish_imu_data(self):
       """Read and publish data from both IMUs"""
       current_time = self.get_clock().now().to_msg()
       
       # Process IMU 1
       imu1_data = self.process_imu1(current_time)
       
       # Process IMU 2
       imu2_data = self.process_imu2(current_time)
       
       # Optionally: Fuse the IMU data and publish combined data
       if imu1_data and imu2_data:
           self.publish_combined_data(imu1_data, imu2_data, current_time)

   def process_imu1(self, timestamp):
       """Process and publish data from IMU 1"""
       try:
           # Read data from IMU 1
           accel = self.bno1.acceleration
           gyro = self.bno1.gyro
           quat = self.bno1.quaternion

           # Check for frozen data
           if self.imu1_last_accel == accel:
               self.imu1_frozen_counter += 1
               if self.imu1_frozen_counter > 10:
                   self.get_logger().warn(f'IMU1 data appears frozen for {self.imu1_frozen_counter} iterations')
                   if self.reset_imu1_if_needed():
                       return None
           else:
               self.imu1_frozen_counter = 0  # Reset counter if data changes

           # Update last values
           self.imu1_last_accel = accel

           # Create IMU message
           imu_msg = Imu()
           imu_msg.header.stamp = timestamp
           imu_msg.header.frame_id = self.imu1_frame

           # Unpack data
           quat_i, quat_j, quat_k, quat_real = quat
           accel_x, accel_y, accel_z = accel
           gyro_x, gyro_y, gyro_z = gyro

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

           # Standard covariance
           imu_msg.orientation_covariance = [0.01] * 9
           imu_msg.angular_velocity_covariance = [0.01] * 9
           imu_msg.linear_acceleration_covariance = [0.01] * 9

           # Publish IMU data
           self.imu1_pub.publish(imu_msg)

           # Create and publish transform
           t = TransformStamped()
           t.header.stamp = timestamp
           t.header.frame_id = self.world_frame
           t.child_frame_id = self.imu1_frame

           # Set translation (fixed position)
           t.transform.translation.x = 0.0
           t.transform.translation.y = 0.0
           t.transform.translation.z = 0.0

           # Set rotation
           t.transform.rotation = imu_msg.orientation

           # Broadcast transform
           self.tf_broadcaster.sendTransform(t)

           # Log data (throttled to reduce spam)
           if not self.in_startup_calibration:  # Don't log data during calibration
               self.get_logger().info(
                   f'Publishing IMU1 data:\n'
                   f'  Quaternion: [{quat_real:.2f}, {quat_i:.2f}, {quat_j:.2f}, {quat_k:.2f}]\n'
                   f'  Angular velocity: [{gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}]\n'
                   f'  Acceleration: [{accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}]',
                   throttle_duration_sec=1.0
               )

           # Reset error counter on success
           self.imu1_error_counter = 0
           
           return imu_msg

       except Exception as e:
           self.imu1_error_counter += 1
           self.get_logger().error(f'Error reading IMU1: {e}')

           if self.imu1_error_counter > 5:
               self.get_logger().error(f'Too many consecutive errors ({self.imu1_error_counter}), resetting IMU1')
               self.reset_imu1_if_needed()
               
           return None
           
   def process_imu2(self, timestamp):
       """Process and publish data from IMU 2"""
       try:
           # Read data from IMU 2
           accel = self.bno2.acceleration
           gyro = self.bno2.gyro
           quat = self.bno2.quaternion

           # Check for frozen data
           if self.imu2_last_accel == accel:
               self.imu2_frozen_counter += 1
               if self.imu2_frozen_counter > 10:
                   self.get_logger().warn(f'IMU2 data appears frozen for {self.imu2_frozen_counter} iterations')
                   if self.reset_imu2_if_needed():
                       return None
           else:
               self.imu2_frozen_counter = 0  # Reset counter if data changes

           # Update last values
           self.imu2_last_accel = accel

           # Create IMU message
           imu_msg = Imu()
           imu_msg.header.stamp = timestamp
           imu_msg.header.frame_id = self.imu2_frame

           # Unpack data
           quat_i, quat_j, quat_k, quat_real = quat
           accel_x, accel_y, accel_z = accel
           gyro_x, gyro_y, gyro_z = gyro

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

           # Standard covariance
           imu_msg.orientation_covariance = [0.01] * 9
           imu_msg.angular_velocity_covariance = [0.01] * 9
           imu_msg.linear_acceleration_covariance = [0.01] * 9

           # Publish IMU data
           self.imu2_pub.publish(imu_msg)

           # Create and publish transform
           t = TransformStamped()
           t.header.stamp = timestamp
           t.header.frame_id = self.world_frame
           t.child_frame_id = self.imu2_frame

           # Set translation based on the IMU2 position relative to world frame
           # Example: If IMU2 is mounted 0.5m from IMU1 along the bar
           t.transform.translation.x = 0.5  # Adjust based on your actual setup
           t.transform.translation.y = 0.0
           t.transform.translation.z = 0.0

           # Set rotation
           t.transform.rotation = imu_msg.orientation

           # Broadcast transform
           self.tf_broadcaster.sendTransform(t)

           # Log data (throttled to reduce spam)
           if not self.in_startup_calibration:  # Don't log data during calibration
               self.get_logger().info(
                   f'Publishing IMU2 data:\n'
                   f'  Quaternion: [{quat_real:.2f}, {quat_i:.2f}, {quat_j:.2f}, {quat_k:.2f}]\n'
                   f'  Angular velocity: [{gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}]\n'
                   f'  Acceleration: [{accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}]',
                   throttle_duration_sec=1.0
               )

           # Reset error counter on success
           self.imu2_error_counter = 0
           
           return imu_msg

       except Exception as e:
           self.imu2_error_counter += 1
           self.get_logger().error(f'Error reading IMU2: {e}')

           if self.imu2_error_counter > 5:
               self.get_logger().error(f'Too many consecutive errors ({self.imu2_error_counter}), resetting IMU2')
               self.reset_imu2_if_needed()
               
           return None
           
   def publish_combined_data(self, imu1_msg, imu2_msg, timestamp):
       """Combine data from both IMUs and publish the fused result"""
       if not imu1_msg or not imu2_msg:
           return  # Skip fusion if either IMU data is missing
           
       # Create a new IMU message for the combined data
       combined_msg = Imu()
       combined_msg.header.stamp = timestamp
       combined_msg.header.frame_id = self.world_frame  # Use world frame for combined data
       
       # Simple averaging approach for orientation
       # In a more sophisticated implementation, you might use a proper sensor fusion algorithm
       w = (imu1_msg.orientation.w + imu2_msg.orientation.w) / 2.0
       x = (imu1_msg.orientation.x + imu2_msg.orientation.x) / 2.0
       y = (imu1_msg.orientation.y + imu2_msg.orientation.y) / 2.0
       z = (imu1_msg.orientation.z + imu2_msg.orientation.z) / 2.0
       
       # Normalize the quaternion
       magnitude = math.sqrt(w*w + x*x + y*y + z*z)
       combined_msg.orientation.w = w / magnitude
       combined_msg.orientation.x = x / magnitude
       combined_msg.orientation.y = y / magnitude
       combined_msg.orientation.z = z / magnitude
       
       # Average the angular velocities
       combined_msg.angular_velocity.x = (imu1_msg.angular_velocity.x +
       # Average the angular velocities
       combined_msg.angular_velocity.x = (imu1_msg.angular_velocity.x + imu2_msg.angular_velocity.x) / 2.0
       combined_msg.angular_velocity.y = (imu1_msg.angular_velocity.y + imu2_msg.angular_velocity.y) / 2.0
       combined_msg.angular_velocity.z = (imu1_msg.angular_velocity.z + imu2_msg.angular_velocity.z) / 2.0
       
       # Average the linear accelerations
       combined_msg.linear_acceleration.x = (imu1_msg.linear_acceleration.x + imu2_msg.linear_acceleration.x) / 2.0
       combined_msg.linear_acceleration.y = (imu1_msg.linear_acceleration.y + imu2_msg.linear_acceleration.y) / 2.0
       combined_msg.linear_acceleration.z = (imu1_msg.linear_acceleration.z + imu2_msg.linear_acceleration.z) / 2.0
       
       # The combined reading should have lower covariance (higher confidence)
       combined_msg.orientation_covariance = [0.005] * 9
       combined_msg.angular_velocity_covariance = [0.005] * 9
       combined_msg.linear_acceleration_covariance = [0.005] * 9
       
       # Publish the combined data
       self.combined_pub.publish(combined_msg)
       
       # Log the combined data occasionally
       if not self.in_startup_calibration:
           self.get_logger().info(
               f'Publishing combined IMU data (fusion of IMU1 and IMU2)',
               throttle_duration_sec=2.0
           )


def main(args=None):
   rclpy.init(args=args)
   node = MultiIMUNode()

   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
   main()
