#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
import pyproj
import math

class GpsToOdom(Node):
    def __init__(self):
        super().__init__('gps_to_odom_node')
        
        # Parameters
        self.declare_parameter('use_utm', True)
        self.declare_parameter('utm_zone', 32)  # Default to 32 for Denmark
        self.declare_parameter('datum_lat', 0.0)
        self.declare_parameter('datum_lon', 0.0)
        self.declare_parameter('position_covariance_rtk', 0.0025)  # 5cm accuracy squared
        self.declare_parameter('position_covariance_gps', 5.0)     # ~2.2m accuracy squared
        self.declare_parameter('use_rtk_covariance', True)         # Whether to use RTK accuracy
        
        self.use_utm = self.get_parameter('use_utm').value
        self.utm_zone = self.get_parameter('utm_zone').value
        
        # Initial datum
        self.datum_lat = self.get_parameter('datum_lat').value
        self.datum_lon = self.get_parameter('datum_lon').value
        self.datum_set = False if (self.datum_lat == 0.0 and self.datum_lon == 0.0) else True
        
        # Covariance parameters
        self.position_covariance_rtk = self.get_parameter('position_covariance_rtk').value
        self.position_covariance_gps = self.get_parameter('position_covariance_gps').value
        self.use_rtk_covariance = self.get_parameter('use_rtk_covariance').value
        
        # Choose covariance based on RTK setting
        self.position_covariance = self.position_covariance_rtk if self.use_rtk_covariance else self.position_covariance_gps
        
        # UTM converter - now configurable by zone
        self.utm_converter = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        
        # Log the UTM zone being used
        self.get_logger().info(f"Using UTM zone {self.utm_zone} for GPS conversion")
        self.get_logger().info(f"Position covariance set to {self.position_covariance} (RTK: {self.use_rtk_covariance})")
        
        # Previous position for velocity calculation
        self.prev_pos = None
        self.prev_time = None
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odometry/gps', 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps_data', self.gps_callback, 10)
        
        self.get_logger().info('GPS to Odometry converter node started')
    
    def gps_callback(self, msg):
        if not msg:
            return
            
        # Set datum for first reading if not set
        if not self.datum_set:
            self.datum_lat = msg.latitude
            self.datum_lon = msg.longitude
            self.datum_set = True
            self.get_logger().info(f'Setting datum to: {self.datum_lat}, {self.datum_lon}')
        
        # Convert GPS to local coordinates
        x, y = self.convert_gps_to_local(msg.latitude, msg.longitude)
        
        # Create odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0 if msg.altitude == 0.0 else msg.altitude
        
        # Orientation (identity quaternion since GPS doesn't provide orientation)
        odom.pose.pose.orientation.w = 1.0
        
        # Calculate velocity if we have previous readings
        current_time = self.get_clock().now()
        if self.prev_pos is not None and self.prev_time is not None:
            dt = (current_time.nanoseconds - self.prev_time.nanoseconds) / 1e9
            if dt > 0:
                vx = (x - self.prev_pos[0]) / dt
                vy = (y - self.prev_pos[1]) / dt
                
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
        
        self.prev_pos = (x, y)
        self.prev_time = current_time
        
        # Use the configured covariance (RTK or GPS)
        pos_cov = self.position_covariance
        odom.pose.covariance[0] = pos_cov  # x
        odom.pose.covariance[7] = pos_cov  # y
        odom.pose.covariance[14] = pos_cov * 2.0  # z (less certain)
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        self.publish_tf(odom)
    
    def convert_gps_to_local(self, lat, lon):
        if self.use_utm:
            # Convert to UTM
            east, north = self.utm_converter(lon, lat)
            
            # Convert to local frame relative to datum
            datum_east, datum_north = self.utm_converter(self.datum_lon, self.datum_lat)
            
            x = east - datum_east
            y = north - datum_north
        else:
            # Simplified conversion using equirectangular projection
            # Note: This is less accurate for larger distances
            earth_radius = 6371000.0  # meters
            lat_rad = math.radians(lat)
            datum_lat_rad = math.radians(self.datum_lat)
            
            x = earth_radius * math.cos(datum_lat_rad) * math.radians(lon - self.datum_lon)
            y = earth_radius * math.radians(lat - self.datum_lat)
        
        return x, y
    
    def publish_tf(self, odom_msg):
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GpsToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
