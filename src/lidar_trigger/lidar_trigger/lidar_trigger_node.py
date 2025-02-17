#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import smbus2
from collections import deque
import statistics

class LidarTriggerNode(Node):
    def __init__(self):
        super().__init__('lidar_trigger_node')
        self.publisher_ = self.create_publisher(Empty, 'trigger', 10)
        self.timer = self.create_timer(0.1, self.check_lidar)

        # LiDAR setup
        self.bus = smbus2.SMBus(1)
        self.tf_address = 0x10          # I2C address of the LiDAR sensor
        self.threshold = 30             # Distance threshold in cm

        # Use a buffer of 5 readings (adjust as needed)
        self.buffer_size = 5
        self.readings_buffer = deque(maxlen=self.buffer_size)
        
        # Option to ignore 0 readings (set to True if 0 is considered invalid)
        self.ignore_zero = True

        self.triggered = False

    def read_lidar(self):
        try:
            # Read 2 bytes from the sensor (adjust command/register if needed)
            data = self.bus.read_i2c_block_data(self.tf_address, 0x00, 2)
            distance = data[0] + (data[1] << 8)
            return distance
        except Exception as e:
            self.get_logger().error(f"Lidar read error: {e}")
            return None

    def check_lidar(self):
        distance = self.read_lidar()
        if distance is None:
            return

        # Log the individual reading
        self.get_logger().debug(f"New reading: {distance} cm")
        
        # Optionally ignore 0 readings if they are known to be spurious
        if self.ignore_zero and distance == 0:
            self.get_logger().warning("Received 0 cm reading; ignoring this value.")
            return

        # Add to buffer and log its contents
        self.readings_buffer.append(distance)
        self.get_logger().debug(f"Buffer contents: {list(self.readings_buffer)}")

        # Only compute the median if the buffer is full
        if len(self.readings_buffer) == self.buffer_size:
            median_distance = statistics.median(self.readings_buffer)
            self.get_logger().debug(f"Computed median: {median_distance:.2f} cm")

            # Log the difference between the last two readings for extra insight
            if len(self.readings_buffer) > 1:
                diff = abs(self.readings_buffer[-1] - self.readings_buffer[-2])
                self.get_logger().debug(f"Difference between last two readings: {diff} cm")

            # Check trigger conditions and log the event
            if median_distance < self.threshold and not self.triggered:
                self.get_logger().info(
                    f"Triggering event: Median {median_distance:.2f} cm is below threshold {self.threshold} cm")
                msg = Empty()
                self.publisher_.publish(msg)
                self.triggered = True
            elif median_distance >= self.threshold and self.triggered:
                self.get_logger().info(
                    f"Reset event: Median {median_distance:.2f} cm is above threshold {self.threshold} cm")
                msg = Empty()
                self.publisher_.publish(msg)
                self.triggered = False

def main(args=None):
    rclpy.init(args=args)
    node = LidarTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
