#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import smbus2
import time

class LidarTriggerNode(Node):
    def __init__(self):
        super().__init__('lidar_trigger_node')
        self.publisher_ = self.create_publisher(Empty, 'trigger', 10)
        # Use create_timer instead of create_wall_timer
        self.timer = self.create_timer(0.1, self.check_lidar)

        # LiDAR setup
        self.bus = smbus2.SMBus(1)
        self.tf_address = 0x10           # I2C address of LiDAR sensor
        self.threshold = 30              # Distance threshold in cm
        self.consecutive_count = 10      # How many consecutive readings below threshold trigger the event

        self.below_threshold_count = 0
        self.above_threshold_count = 0
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
        if distance is not None:
            self.get_logger().debug(f"Lidar distance: {distance} cm")
            if distance < self.threshold:
                self.below_threshold_count += 1
                self.above_threshold_count = 0
            else:
                self.above_threshold_count += 1
                self.below_threshold_count = 0

            if self.below_threshold_count >= self.consecutive_count and not self.triggered:
                self.get_logger().info("LiDAR trigger condition met. Publishing trigger message.")
                msg = Empty()
                self.publisher_.publish(msg)
                self.triggered = True
            elif self.above_threshold_count >= self.consecutive_count and self.triggered:
                self.get_logger().info("LiDAR trigger reset.")
                self.triggered = False

def main(args=None):
    rclpy.init(args=args)
    node = LidarTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
