#!/usr/bin/env python3
import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

print("BNO085 Calibration Monitor")
print("==========================")
print("Follow these steps for each sensor:")
print("1. Accelerometer: Place the IMU in 6 different static positions for a few seconds each.")
print("   - Make sure to include positions with the IMU facing up, down, left, right, forward and backward")
print("2. Gyroscope: Let the IMU sit still on a stable surface for 10 seconds")
print("3. Magnetometer: Move the IMU in a figure-8 pattern in the air several times.")
print("\nStarting calibration monitoring. Press Ctrl+C to exit.")

try:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    
    # Enable all reports needed
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    
    # Wait a bit for values to stabilize
    time.sleep(1)
    
    while True:
        # Get the rotation vector (which has accuracy info)
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        
        # Get sensor readings
        accel_x, accel_y, accel_z = bno.acceleration
        gyro_x, gyro_y, gyro_z = bno.gyro
        mag_x, mag_y, mag_z = bno.magnetic
        
        print("\033c", end="")  # Clear the terminal
        print("BNO085 CALIBRATION STATUS")
        print("=========================")
        
        # Print current readings
        print(f"Acceleration (m/s²): X: {accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f}")
        print(f"Gyroscope (rad/s): X: {gyro_x:.2f}, Y: {gyro_y:.2f}, Z: {gyro_z:.2f}")
        print(f"Magnetic (uT): X: {mag_x:.2f}, Y: {mag_y:.2f}, Z: {mag_z:.2f}")
        print(f"Quaternion: W: {quat_real:.2f}, X: {quat_i:.2f}, Y: {quat_j:.2f}, Z: {quat_k:.2f}")
        
        print("\nCALIBRATION GUIDE:")
        print("1. Accelerometer: Hold IMU in 6 different static positions for a few seconds each")
        print("2. Gyroscope: Keep IMU still on a flat surface")
        print("3. Magnetometer: Move IMU in a figure-8 pattern in the air")
        
        time.sleep(0.1)  # Update 10 times per second
        
except KeyboardInterrupt:
    print("\nCalibration monitor stopped.")
except Exception as e:
    print(f"Error: {e}")
