#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import os
import json
from datetime import datetime

class GPSVisualizerNode(Node):
    def __init__(self):
        super().__init__('gps_visualizer_node')
        
        # Create publishers for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'gps_path_markers', 10)
        
        # Subscribe to GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10)
            
        # Subscribe to filtered GPS if available
        self.filtered_gps_sub = self.create_subscription(
            NavSatFix,
            'gps/filtered',
            self.filtered_gps_callback,
            10)
        
        # Data storage
        self.raw_points = []
        self.filtered_points = []
        
        # Parameters
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('save_path', True)
        self.declare_parameter('save_interval', 5.0)
        
        self.max_points = self.get_parameter('max_points').value
        self.save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # Setup data saving
        if self.save_path:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_dir = os.path.join(os.getcwd(), 'path_logs')
            os.makedirs(self.log_dir, exist_ok=True)
            self.log_file = os.path.join(self.log_dir, f'gps_path_{timestamp}.json')
            
            # Initialize save timer
            self.save_timer = self.create_timer(self.save_interval, self.save_path_data)
            
            self.get_logger().info(f'Path data will be saved to: {self.log_file}')
        
        # Timer for visualization update
        self.vis_timer = self.create_timer(0.2, self.publish_visualization)
        
        self.get_logger().info('GPS Visualizer node initialized')
    
    def gps_callback(self, msg):
        # Convert lat/lon to simplified local coordinates
        # This is just for visualization - we'll use a simple equirectangular projection
        # centered at the first point we receive
        
        if not self.raw_points:
            # First point becomes the origin
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
        
        # Simple conversion to meters (approximate)
        x = (msg.longitude - self.origin_lon) * 111320.0 * \
            float(0.0 if self.origin_lat == 90.0 else 
                (1.0 if self.origin_lat == 0.0 else 
                 (0.0 if self.origin_lat == -90.0 else 
                   (1.0 if abs(self.origin_lat) == 0.0 else 
                       (1.0 / abs(self.origin_lat) * 3.1412 / 180.0))
                  )
                )
              )
        y = (msg.latitude - self.origin_lat) * 111320.0
        
        point = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'x': x,
            'y': y,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        self.raw_points.append(point)
        
        # Limit points
        if len(self.raw_points) > self.max_points:
            self.raw_points = self.raw_points[-self.max_points:]
    
    def filtered_gps_callback(self, msg):
        if not hasattr(self, 'origin_lat'):
            # Wait until we have raw points first
            return
            
        # Use same conversion as for raw points
        x = (msg.longitude - self.origin_lon) * 111320.0 * \
            float(0.0 if self.origin_lat == 90.0 else 
                (1.0 if self.origin_lat == 0.0 else 
                 (0.0 if self.origin_lat == -90.0 else 
                   (1.0 if abs(self.origin_lat) == 0.0 else 
                       (1.0 / abs(self.origin_lat) * 3.1412 / 180.0))
                  )
                )
              )
        y = (msg.latitude - self.origin_lat) * 111320.0
        
        point = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'x': x,
            'y': y,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        self.filtered_points.append(point)
        
        # Limit points
        if len(self.filtered_points) > self.max_points:
            self.filtered_points = self.filtered_points[-self.max_points:]
    
    def publish_visualization(self):
        if not self.raw_points:
            return
            
        marker_array = MarkerArray()
        
        # Create path line for raw GPS
        raw_line = Marker()
        raw_line.header.frame_id = "map"
        raw_line.header.stamp = self.get_clock().now().to_msg()
        raw_line.ns = "gps_paths"
        raw_line.id = 0
        raw_line.type = Marker.LINE_STRIP
        raw_line.action = Marker.ADD
        raw_line.pose.orientation.w = 1.0
        raw_line.scale.x = 0.2  # Line width
        raw_line.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        
        for point in self.raw_points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            raw_line.points.append(p)
        
        marker_array.markers.append(raw_line)
        
        # Create path line for filtered GPS (if available)
        if self.filtered_points:
            filtered_line = Marker()
            filtered_line.header.frame_id = "map"
            filtered_line.header.stamp = self.get_clock().now().to_msg()
            filtered_line.ns = "gps_paths"
            filtered_line.id = 1
            filtered_line.type = Marker.LINE_STRIP
            filtered_line.action = Marker.ADD
            filtered_line.pose.orientation.w = 1.0
            filtered_line.scale.x = 0.2  # Line width
            filtered_line.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue
            
            for point in self.filtered_points:
                p = Point()
                p.x = point['x']
                p.y = point['y']
                p.z = 0.0
                filtered_line.points.append(p)
            
            marker_array.markers.append(filtered_line)
            
            # Add current position marker
            current_pos = Marker()
            current_pos.header.frame_id = "map"
            current_pos.header.stamp = self.get_clock().now().to_msg()
            current_pos.ns = "gps_paths"
            current_pos.id = 2
            current_pos.type = Marker.SPHERE
            current_pos.action = Marker.ADD
            current_pos.pose.position.x = self.filtered_points[-1]['x']
            current_pos.pose.position.y = self.filtered_points[-1]['y']
            current_pos.pose.position.z = 0.0
            current_pos.pose.orientation.w = 1.0
            current_pos.scale.x = 0.5
            current_pos.scale.y = 0.5
            current_pos.scale.z = 0.5
            current_pos.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            
            marker_array.markers.append(current_pos)
        
        # Publish the marker array
        self.marker_pub.publish(marker_array)
    
    def save_path_data(self):
        if not self.raw_points and not self.filtered_points:
            return
            
        # Create a dictionary with both paths
        path_data = {
            "timestamp": datetime.now().isoformat(),
            "origin": {
                "latitude": self.origin_lat,
                "longitude": self.origin_lon
            },
            "raw_path": self.raw_points,
            "filtered_path": self.filtered_points
        }
        
        # Save to file
        with open(self.log_file, 'w') as f:
            json.dump(path_data, f, indent=2)
            
        self.get_logger().info(f'Saved path data to: {self.log_file}')
        
def main():
    rclpy.init()
    node = GPSVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
