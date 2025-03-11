#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import time
from datetime import datetime

class GPSSimulatorNode(Node):
    def __init__(self):
        super().__init__('gps_simulator_node')
        
        # Create publisher for GPS data
        self.gps_pub = self.create_publisher(NavSatFix, 'gps_data', 10)
        
        # Parameters
        self.declare_parameter('center_lat', 59.9127)  # Stockholm
        self.declare_parameter('center_lon', 10.7461)  # Oslo
        self.declare_parameter('radius', 10.0)         # Circle radius in meters
        self.declare_parameter('speed', 1.0)           # Movement speed in m/s
        self.declare_parameter('freq', 1.0)            # Update frequency in Hz
        
        # Get parameters
        self.center_lat = self.get_parameter('center_lat').value
        self.center_lon = self.get_parameter('center_lon').value
        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        self.freq = self.get_parameter('freq').value
        
        # Variables for motion
        self.angle = 0.0
        self.start_time = time.time()
        
        # Create timer for publishing updates
        self.timer = self.create_timer(1.0/self.freq, self.publish_gps)
        
        self.get_logger().info('GPS Simulator started, publishing to "gps_data" topic')
        
    def publish_gps(self):
        # Calculate elapsed time
        elapsed = time.time() - self.start_time
        
        # Simple constant angular velocity motion
        angular_velocity = self.speed / self.radius  # rad/s
        self.angle = (elapsed * angular_velocity) % (2.0 * math.pi)
        
        # Calculate position on circle
        lat_offset = self.meters_to_lat(self.radius * math.sin(self.angle))
        lon_offset = self.meters_to_lon(self.radius * math.cos(self.angle), self.center_lat)
        
        # Create GPS message
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.latitude = self.center_lat + lat_offset
        msg.longitude = self.center_lon + lon_offset
        msg.altitude = 0.0
        
        # Set covariance based on simulated "quality"
        # Lower values for better fix - we'll simulate a good RTK fix
        position_covariance = 0.01  # 10cm accuracy
        msg.position_covariance = [position_covariance, 0.0, 0.0,
                                  0.0, position_covariance, 0.0,
                                  0.0, 0.0, position_covariance * 4.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Publish
        self.gps_pub.publish(msg)
        self.get_logger().info(f'Publishing simulated GPS: {msg.latitude}, {msg.longitude}')

    def meters_to_lat(self, meters):
        # Approximate conversion from meters to latitude degrees
        return meters / 111320.0
    
    def meters_to_lon(self, meters, lat):
        # Approximate conversion from meters to longitude degrees
        # This varies with latitude
        return meters / (111320.0 * math.cos(math.radians(lat)))

def main():
    rclpy.init()
    node = GPSSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
