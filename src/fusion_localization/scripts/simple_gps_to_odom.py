#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import pyproj
import math

class SimpleGpsToOdom(Node):
    def __init__(self):
        super().__init__('simple_gps_to_odom')
        
        # Create a Universal Transverse Mercator projection
        self.utm_proj = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')
        
        # First point is the reference point
        self.ref_x = None
        self.ref_y = None
        
        # Create a subscriber for GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )
        
        # Create a publisher for odometry data
        self.odom_pub = self.create_publisher(
            Odometry,
            'odometry/gps',
            10
        )
    
    def gps_callback(self, msg):
        # Convert lat/lon to UTM coordinates
        x, y = self.utm_proj(msg.longitude, msg.latitude)
        
        # Set reference point if not set
        if self.ref_x is None or self.ref_y is None:
            self.ref_x = x
            self.ref_y = y
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position relative to reference
        odom.pose.pose.position.x = x - self.ref_x
        odom.pose.pose.position.y = y - self.ref_y
        odom.pose.pose.position.z = 0.0
        
        # Identity orientation (no orientation from GPS)
        odom.pose.pose.orientation.w = 1.0
        
        # Copy covariance
        odom.pose.covariance[0] = msg.position_covariance[0]  # x
        odom.pose.covariance[7] = msg.position_covariance[4]  # y
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        self.get_logger().info(f'Published odometry: x={odom.pose.pose.position.x:.2f}, y={odom.pose.pose.position.y:.2f}')

def main():
    rclpy.init()
    node = SimpleGpsToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
