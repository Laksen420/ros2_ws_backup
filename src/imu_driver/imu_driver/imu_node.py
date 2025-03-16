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

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        #declare params with calibration off by default.
        self.declare_parameter('enable_startup_calibration', False)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('imu_frame', 'imu_link')

        # Get parameter values
        self.in_startup_calibration = self.get_parameter('enable_startup_calibration').value
        self.world_frame = self.get_parameter('world_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.calib_pub = self.create_publisher(String, 'imu/calibration_status', 10)
        self.calib_complete_pub = self.create_publisher(Bool, 'imu/calibration_complete', 10)

        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frame names
        self.world_frame = "world"
        self.imu_frame = "imu_link"

        # Calibration parameters
        self.in_startup_calibration = True
        self.gyro_samples = []
        self.accel_samples = []
        self.mag_samples = []
        self.calibration_timeout = 30.0  # 30 seconds for initial calibration
        self.calibration_start_time = None

        # Frozen data detection
        self.last_accel = None
        self.frozen_counter = 0
        self.error_counter = 0

        # Initialize IMU
        self.init_imu()

        # Create timers
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 20Hz IMU data
        self.calib_timer = self.create_timer(1.0, self.publish_calibration_status)  # 1Hz calibration status

        # Start calibration
        self.calibration_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info('IMU Node started - beginning calibration sequence')
        self.get_logger().info('For best results during calibration:')
        self.get_logger().info('1. Keep the platform still for the first 10 seconds')
        self.get_logger().info('2. Then slowly rotate/tilt the platform in different directions')
        self.get_logger().info('3. Finally, drive in a figure-8 pattern if possible')

    def init_imu(self):
        """Initialize the IMU"""
        try:
            self.get_logger().info('Initializing BNO085 IMU...')

            # Create I2C bus
            self.i2c = busio.I2C(board.SCL, board.SDA)
            time.sleep(0.5)  # Wait for I2C to initialize

            # Create IMU object
            self.bno = BNO08X_I2C(self.i2c)

            # Enable features
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            # Reset counters
            self.frozen_counter = 0
            self.error_counter = 0

            # Reset calibration data
            self.gyro_samples = []
            self.accel_samples = []
            self.mag_samples = []

            self.get_logger().info('BNO085 initialized successfully!')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to initialize IMU: {e}')
            return False

    def reset_if_needed(self):
        """Reset the IMU if too many errors or frozen data"""
        self.get_logger().warn('Resetting IMU...')
        try:
            del self.bno
            del self.i2c
        except:
            pass

        time.sleep(1.0)  # Wait before reinitializing
        return self.init_imu()

    def evaluate_calibration(self):
        """Evaluate the calibration quality based on collected samples"""
        if not self.gyro_samples or not self.accel_samples or not self.mag_samples:
            return False, "Insufficient data for calibration evaluation"

        # Check gyro stability (should be close to zero when stationary)
        gyro_avg = [sum(axis) / len(axis) for axis in zip(*self.gyro_samples)]
        gyro_std = [
            math.sqrt(sum((val - gyro_avg[i])**2 for val in axis) / len(axis))
            for i, axis in enumerate(zip(*self.gyro_samples))
        ]
        gyro_ok = all(std < 0.05 for std in gyro_std)

        # Check accelerometer (magnitude should be close to g = 9.81 m/s²)
        accel_magnitudes = [
            math.sqrt(x**2 + y**2 + z**2)
            for x, y, z in self.accel_samples
        ]
        accel_avg_mag = sum(accel_magnitudes) / len(accel_magnitudes)
        accel_ok = abs(accel_avg_mag - 9.81) < 0.5

        # Check magnetometer (should have reasonable variation for calibration)
        mag_min = [min(axis) for axis in zip(*self.mag_samples)]
        mag_max = [max(axis) for axis in zip(*self.mag_samples)]
        mag_range = [max_val - min_val for min_val, max_val in zip(mag_min, mag_max)]
        mag_ok = all(rng > 5.0 for rng in mag_range)  # Should have at least 5μT range in each axis

        # Overall calibration status
        calib_ok = gyro_ok and accel_ok and mag_ok

        status_msg = (
            f"Gyroscope: {'OK' if gyro_ok else 'Needs calibration'} (std: {gyro_std})\n"
            f"Accelerometer: {'OK' if accel_ok else 'Needs calibration'} (avg mag: {accel_avg_mag:.2f})\n"
            f"Magnetometer: {'OK' if mag_ok else 'Needs calibration'} (range: {mag_range})"
        )

        return calib_ok, status_msg

    def process_startup_calibration(self):
        """Process the startup calibration sequence"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.calibration_start_time

        # Collect sensor data for calibration evaluation
        try:
            gyro = self.bno.gyro
            accel = self.bno.acceleration
            mag = self.bno.magnetic

            self.gyro_samples.append(gyro)
            self.accel_samples.append(accel)
            self.mag_samples.append(mag)

            # Limit sample size to avoid memory issues
            max_samples = 100
            if len(self.gyro_samples) > max_samples:
                self.gyro_samples.pop(0)
            if len(self.accel_samples) > max_samples:
                self.accel_samples.pop(0)
            if len(self.mag_samples) > max_samples:
                self.mag_samples.pop(0)

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
                calib_ok, status = self.evaluate_calibration()

                # Publish calibration complete message
                complete_msg = Bool()
                complete_msg.data = calib_ok
                self.calib_complete_pub.publish(complete_msg)

                if calib_ok:
                    self.get_logger().info("Calibration complete and successful!")
                else:
                    self.get_logger().warn("Calibration complete but some sensors may need more calibration")
                    self.get_logger().warn(status)

                self.in_startup_calibration = False
                return

            # Update calibration status
            calib_msg = String()
            calib_msg.data = (
                f"CALIBRATION IN PROGRESS: {elapsed_time:.1f}/{self.calibration_timeout:.1f}s\n"
                f"{phase_msg}\n"
                f"Samples Collected: {len(self.gyro_samples)}"
            )
            self.calib_pub.publish(calib_msg)

        except Exception as e:
            self.get_logger().error(f"Error during calibration: {e}")

    def publish_calibration_status(self):
        """Publish calibration status information"""
        # During startup calibration, handle differently
        if self.in_startup_calibration:
            self.process_startup_calibration()
            return

        try:
            # Get readings to assess calibration
            accel = self.bno.acceleration
            gyro = self.bno.gyro
            mag = self.bno.magnetic

            # Approximate calibration status based on reading characteristics
            # (BNO085 doesn't directly provide calibration status like BNO055)

            # For gyro, check if values are close to zero when stationary
            gyro_calibrated = all(abs(v) < 0.05 for v in gyro)

            # For accelerometer, check if magnitude is close to 9.8 m/s²
            accel_magnitude = (accel[0]**2 + accel[1]**2 + accel[2]**2)**0.5
            accel_calibrated = abs(accel_magnitude - 9.81) < 0.5

            # For magnetometer, check if values are reasonable
            mag_magnitude = (mag[0]**2 + mag[1]**2 + mag[2]**2)**0.5
            mag_calibrated = 25 < mag_magnitude < 65  # Typical range for Earth's magnetic field

            # Create calibration message
            calib_msg = String()
            calib_msg.data = (
                f"Calibration Status (approximate):\n"
                f"Accelerometer: {'Calibrated' if accel_calibrated else 'Needs Calibration'}\n"
                f"Gyroscope: {'Calibrated' if gyro_calibrated else 'Needs Calibration'}\n"
                f"Magnetometer: {'Calibrated' if mag_calibrated else 'Needs Calibration'}\n"
                f"Accel magnitude: {accel_magnitude:.2f} m/s² (should be ~9.8)\n"
                f"Mag magnitude: {mag_magnitude:.2f} μT (should be ~25-65)"
            )

            self.calib_pub.publish(calib_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing calibration status: {e}')

    def publish_imu_data(self):
        """Read and publish IMU data"""
        try:
            # Read data from IMU
            accel = self.bno.acceleration
            gyro = self.bno.gyro
            quat = self.bno.quaternion

            # Check for frozen data
            if self.last_accel == accel:
                self.frozen_counter += 1
                if self.frozen_counter > 10:
                    self.get_logger().warn(f'IMU data appears frozen for {self.frozen_counter} iterations')
                    if self.reset_if_needed():
                        return
            else:
                self.frozen_counter = 0  # Reset counter if data changes

            # Update last values
            self.last_accel = accel

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame

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
            self.imu_pub.publish(imu_msg)

            # Create and publish transform
            t = TransformStamped()
            t.header.stamp = imu_msg.header.stamp
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.imu_frame

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
                    f'Publishing IMU data:\n'
                    f'  Quaternion: [{quat_real:.2f}, {quat_i:.2f}, {quat_j:.2f}, {quat_k:.2f}]\n'
                    f'  Angular velocity: [{gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}]\n'
                    f'  Acceleration: [{accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}]',
                    throttle_duration_sec=1.0
                )

            # Reset error counter on success
            self.error_counter = 0

        except Exception as e:
            self.error_counter += 1
            self.get_logger().error(f'Error reading IMU: {e}')

            if self.error_counter > 5:
                self.get_logger().error(f'Too many consecutive errors ({self.error_counter}), resetting IMU')
                self.reset_if_needed()

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
