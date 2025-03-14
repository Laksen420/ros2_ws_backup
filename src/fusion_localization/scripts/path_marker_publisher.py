#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from collections import deque
import math

class PathMarkerPublisher(Node):
    def __init__(self):
        super().__init__('path_marker_publisher')

        # Store GPS points
        self.points = deque(maxlen=1000)  # Limit to 1000 points

        # Subscribe to GPS data
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )

        # Create a publisher for Marker
        self.marker_publisher = self.create_publisher(
            Marker,
            'gps_path_marker',
            10
        )

        # Timer to publish Marker
        self.timer = self.create_timer(1.0, self.publish_marker)

    def gps_callback(self, msg):
        # Store the lat/lon points
        self.points.append((msg.longitude, msg.latitude))
        self.get_logger().info(f"Received GPS point: {msg.longitude}, {msg.latitude}")

    def publish_marker(self):
        if not self.points:
            return

        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"  # This should match your map frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gps_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Size
        marker.scale.x = 0.5  # Line width
        
        # Color (blue)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Add points
        for lon, lat in self.points:
            p = Point()
            p.x = lon
            p.y = lat
            p.z = 0.0
            marker.points.append(p)
        
        # Publish
        self.marker_publisher.publish(marker)
        self.get_logger().info(f'Published path marker with {len(self.points)} points')

def main():
    rclpy.init()
    node = PathMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
