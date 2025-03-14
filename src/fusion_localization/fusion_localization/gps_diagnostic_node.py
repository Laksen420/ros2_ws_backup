#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray
import math
import numpy as np

class GPSDiagnosticNode(Node):
    def __init__(self):
        super().__init__('gps_diagnostic_node')
        
        # Create publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/diagnostic/path_markers', 10)
        
        # Create a publisher for 2D plot data (x, y coordinates)
        self.plot_pub = self.create_publisher(Float32MultiArray, '/diagnostic/path_plot', 10)
        
        # Subscribe to GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10)
        
        # Storage for GPS points
        self.points = []
        self.origin_lat = None
        self.origin_lon = None
        
        # Configure maximum points to store
        self.declare_parameter('max_points', 200)
        self.max_points = self.get_parameter('max_points').value
        
        # Create a timer for visualization updates
        self.vis_timer = self.create_timer(0.1, self.publish_visualization)
        
        # Create a timer for statistics calculation
        self.stats_timer = self.create_timer(5.0, self.calculate_statistics)
        
        self.get_logger().info('GPS Diagnostic Node initialized')
    
    def gps_callback(self, msg):
        """Process incoming GPS data."""
        # Set origin on first point
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'Origin set to: {self.origin_lat}, {self.origin_lon}')
        
        # Convert to local XY coordinates (simple equirectangular projection)
        cos_lat = math.cos(math.radians(self.origin_lat))
        x = (msg.longitude - self.origin_lon) * 111320.0 * cos_lat
        y = (msg.latitude - self.origin_lat) * 111320.0
        
        # Store point data
        point = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'x': x,
            'y': y,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        self.points.append(point)
        
        # Limit the number of stored points
        if len(self.points) > self.max_points:
            self.points = self.points[-self.max_points:]
        
        # Log the point
        self.get_logger().debug(f'New GPS point: x={x:.2f}, y={y:.2f}')
    
    def publish_visualization(self):
        """Publish visualization markers for the GPS path."""
        if not self.points:
            return
        
        # Create marker array for visualization
        marker_array = MarkerArray()
        
        # Points marker (spheres at each GPS position)
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "gps_points"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = 0.5
        points_marker.scale.y = 0.5
        points_marker.scale.z = 0.5
        points_marker.color.r = 1.0
        points_marker.color.g = 0.5
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        
        # Add each point to the marker
        for point in self.points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            points_marker.points.append(p)
        
        marker_array.markers.append(points_marker)
        
        # Path marker (line connecting points)
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "gps_path"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.2  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        
        # Add points to line strip
        for point in self.points:
            p = Point()
            p.x = point['x']
            p.y = point['y']
            p.z = 0.0
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Current position marker (larger sphere at latest position)
        current_marker = Marker()
        current_marker.header.frame_id = "map"
        current_marker.header.stamp = self.get_clock().now().to_msg()
        current_marker.ns = "gps_current"
        current_marker.id = 2
        current_marker.type = Marker.SPHERE
        current_marker.action = Marker.ADD
        current_marker.pose.position.x = self.points[-1]['x']
        current_marker.pose.position.y = self.points[-1]['y']
        current_marker.pose.position.z = 0.0
        current_marker.pose.orientation.w = 1.0
        current_marker.scale.x = 1.0
        current_marker.scale.y = 1.0
        current_marker.scale.z = 1.0
        current_marker.color.r = 0.0
        current_marker.color.g = 1.0
        current_marker.color.b = 0.0
        current_marker.color.a = 1.0
        
        marker_array.markers.append(current_marker)
        
        # Ground plane reference
        plane_marker = Marker()
        plane_marker.header.frame_id = "map"
        plane_marker.header.stamp = self.get_clock().now().to_msg()
        plane_marker.ns = "reference"
        plane_marker.id = 3
        plane_marker.type = Marker.CUBE
        plane_marker.action = Marker.ADD
        plane_marker.pose.position.x = 0.0
        plane_marker.pose.position.y = 0.0
        plane_marker.pose.position.z = -0.1
        plane_marker.pose.orientation.w = 1.0
        plane_marker.scale.x = 40.0
        plane_marker.scale.y = 40.0
        plane_marker.scale.z = 0.05
        plane_marker.color.r = 0.8
        plane_marker.color.g = 0.8
        plane_marker.color.b = 0.8
        plane_marker.color.a = 0.3
        
        marker_array.markers.append(plane_marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
        
        # Publish 2D plot data
        self.publish_plot_data()
    
    def publish_plot_data(self):
        """Publish data for 2D plotting."""
        if not self.points:
            return
        
        # Extract x and y coordinates from points
        x_coords = [p['x'] for p in self.points]
        y_coords = [p['y'] for p in self.points]
        
        # Combine into a flat array for publishing
        # Format: [x1, y1, x2, y2, ...]
        plot_data = []
        for x, y in zip(x_coords, y_coords):
            plot_data.extend([x, y])
        
        # Create and publish Float32MultiArray
        msg = Float32MultiArray()
        msg.data = plot_data
        self.plot_pub.publish(msg)
    
    def calculate_statistics(self):
        """Calculate and log statistics about the GPS path."""
        if len(self.points) < 10:
            return
        
        # Extract x and y coordinates
        x_coords = np.array([p['x'] for p in self.points])
        y_coords = np.array([p['y'] for p in self.points])
        
        # Calculate basic statistics
        x_min, x_max = np.min(x_coords), np.max(x_coords)
        y_min, y_max = np.min(y_coords), np.max(y_coords)
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        # Calculate path length
        total_distance = 0.0
        for i in range(1, len(self.points)):
            dx = self.points[i]['x'] - self.points[i-1]['x']
            dy = self.points[i]['y'] - self.points[i-1]['y']
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        # Log the statistics
        self.get_logger().info(f"GPS Path Statistics:")
        self.get_logger().info(f"  X range: {x_min:.2f} to {x_max:.2f} meters (width: {x_range:.2f}m)")
        self.get_logger().info(f"  Y range: {y_min:.2f} to {y_max:.2f} meters (height: {y_range:.2f}m)")
        self.get_logger().info(f"  Total distance: {total_distance:.2f} meters")
        self.get_logger().info(f"  Number of points: {len(self.points)}")
        
        # Calculate aspect ratio to determine if it's circular or figure-eight
        aspect_ratio = x_range / y_range if y_range > 0 else 0
        
        if aspect_ratio > 1.8:
            self.get_logger().info("  Path shape appears to be figure-eight-like (width > height)")
        elif 0.8 <= aspect_ratio <= 1.2:
            self.get_logger().info("  Path shape appears to be circular (width ≈ height)")
        else:
            self.get_logger().info(f"  Path aspect ratio: {aspect_ratio:.2f} (width/height)")

def main():
    rclpy.init()
    node = GPSDiagnosticNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
