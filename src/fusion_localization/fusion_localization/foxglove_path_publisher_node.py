#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
import json
import os
import math
from datetime import datetime
from tf2_ros import TransformBroadcaster

class FoxglovePathPublisherNode(Node):
    def __init__(self):
        super().__init__('foxglove_path_publisher')

        # Create publishers for visualization in Foxglove
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/gps_path_markers', 10)
        self.path_pub = self.create_publisher(Path, '/visualization/gps_path', 10)
        self.raw_path_pub = self.create_publisher(Path, '/visualization/raw_gps_path', 10)

        # For Foxglove custom format as a fallback
        self.foxglove_pub = self.create_publisher(String, '/foxglove/path_data', 10)
        
        # Set up a transform broadcaster for coordinate frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10)

        # Subscribe to filtered GPS data
        self.filtered_gps_sub = self.create_subscription(
            NavSatFix,
            'gps/filtered',
            self.filtered_gps_callback,
            10)

        # Data storage
        self.raw_points = []
        self.filtered_points = []

        # Initialize Path messages
        self.raw_path = Path()
        self.raw_path.header.frame_id = "map"

        self.filtered_path = Path()
        self.filtered_path.header.frame_id = "map"

        # Parameters
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('save_path', True)
        self.declare_parameter('save_interval', 5.0)
        self.declare_parameter('scale_factor', 1.0)  # Scale factor for visualization

        self.max_points = self.get_parameter('max_points').value
        self.save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        self.scale_factor = self.get_parameter('scale_factor').value

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

        # First GPS point flag
        self.have_first_point = False

        self.get_logger().info('Foxglove Path Publisher initialized')

    def gps_callback(self, msg):
        # First point handling
        if not self.have_first_point:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.have_first_point = True

            self.get_logger().info(f'Origin set to: {self.origin_lat}, {self.origin_lon}')

        # Convert to local coordinates (simplified equirectangular projection)
        cos_lat = math.cos(math.radians(self.origin_lat))
        x = (msg.longitude - self.origin_lon) * 111320.0 * cos_lat
        y = (msg.latitude - self.origin_lat) * 111320.0

        # Apply scaling for better visualization
        viz_x = x * self.scale_factor
        viz_y = y * self.scale_factor

        # Store point data (original coordinates for saving)
        point = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'x': x,
            'y': y,
            'viz_x': viz_x,
            'viz_y': viz_y,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        self.raw_points.append(point)

        # Create pose for path (using visualization coordinates)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = viz_x
        pose.pose.position.y = viz_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Add to raw path
        self.raw_path.poses.append(pose)

        # Limit points
        if len(self.raw_points) > self.max_points:
            self.raw_points = self.raw_points[-self.max_points:]
            self.raw_path.poses = self.raw_path.poses[-self.max_points:]

    def filtered_gps_callback(self, msg):
        if not self.have_first_point:
            return  # Wait for raw GPS to establish origin

        # Convert to local coordinates
        cos_lat = math.cos(math.radians(self.origin_lat))
        x = (msg.longitude - self.origin_lon) * 111320.0 * cos_lat
        y = (msg.latitude - self.origin_lat) * 111320.0

        # Apply scaling for better visualization
        viz_x = x * self.scale_factor
        viz_y = y * self.scale_factor

        # Store point data
        point = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'x': x,
            'y': y,
            'viz_x': viz_x,
            'viz_y': viz_y,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        self.filtered_points.append(point)

        # Create pose for path
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = viz_x
        pose.pose.position.y = viz_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # Add to filtered path
        self.filtered_path.poses.append(pose)

        # Limit points
        if len(self.filtered_points) > self.max_points:
            self.filtered_points = self.filtered_points[-self.max_points:]
            self.filtered_path.poses = self.filtered_path.poses[-self.max_points:]

        # Broadcast the current position as a transform for better visualization
        self.broadcast_transform(viz_x, viz_y, msg.altitude)

    def broadcast_transform(self, x, y, z):
        """Broadcast the current position as a transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "current_position"
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def publish_visualization(self):
        if not self.have_first_point:
            return

        # Update headers with current timestamp
        now = self.get_clock().now().to_msg()

        # Publish Path messages (for normal RViz-like visualizers in Foxglove)
        if self.raw_path.poses:
            self.raw_path.header.stamp = now
            self.raw_path_pub.publish(self.raw_path)

        if self.filtered_path.poses:
            self.filtered_path.header.stamp = now
            self.path_pub.publish(self.filtered_path)

        # Create and publish markers (these are more visible in Foxglove)
        marker_array = MarkerArray()

        # Add a grid reference to the map
        self.add_reference_grid(marker_array, now)

        # Raw path line
        if self.raw_points:
            raw_line = Marker()
            raw_line.header.frame_id = "map"
            raw_line.header.stamp = now
            raw_line.ns = "gps_paths"
            raw_line.id = 0
            raw_line.type = Marker.LINE_STRIP
            raw_line.action = Marker.ADD
            raw_line.pose.orientation.w = 1.0
            raw_line.scale.x = 0.5  # Increased line width for better visibility
            raw_line.color.r = 1.0
            raw_line.color.g = 0.0
            raw_line.color.b = 0.0
            raw_line.color.a = 1.0

            for point in self.raw_points:
                p = Point()
                p.x = point['viz_x']  # Use visualization-scaled coordinates
                p.y = point['viz_y']
                p.z = 0.0
                raw_line.points.append(p)

            marker_array.markers.append(raw_line)

        # Filtered path line
        if self.filtered_points:
            filtered_line = Marker()
            filtered_line.header.frame_id = "map"
            filtered_line.header.stamp = now
            filtered_line.ns = "gps_paths"
            filtered_line.id = 1
            filtered_line.type = Marker.LINE_STRIP
            filtered_line.action = Marker.ADD
            filtered_line.pose.orientation.w = 1.0
            filtered_line.scale.x = 0.5  # Increased line width for better visibility
            filtered_line.color.r = 0.0
            filtered_line.color.g = 0.0
            filtered_line.color.b = 1.0
            filtered_line.color.a = 1.0

            for point in self.filtered_points:
                p = Point()
                p.x = point['viz_x']  # Use visualization-scaled coordinates
                p.y = point['viz_y']
                p.z = 0.0
                filtered_line.points.append(p)

            marker_array.markers.append(filtered_line)

            # Current position marker
            current_pos = Marker()
            current_pos.header.frame_id = "map"
            current_pos.header.stamp = now
            current_pos.ns = "gps_paths"
            current_pos.id = 2
            current_pos.type = Marker.SPHERE
            current_pos.action = Marker.ADD
            current_pos.pose.position.x = self.filtered_points[-1]['viz_x']
            current_pos.pose.position.y = self.filtered_points[-1]['viz_y']
            current_pos.pose.position.z = 0.0
            current_pos.pose.orientation.w = 1.0
            current_pos.scale.x = 2.0  # Larger sphere for better visibility
            current_pos.scale.y = 2.0
            current_pos.scale.z = 2.0
            current_pos.color.r = 0.0
            current_pos.color.g = 1.0
            current_pos.color.b = 0.0
            current_pos.color.a = 1.0

            marker_array.markers.append(current_pos)

        # Publish the marker array
        self.marker_pub.publish(marker_array)

        # Also publish a custom format for Foxglove's map panel
        self.publish_foxglove_data()

    def add_reference_grid(self, marker_array, now):
        """Add reference grid markers to help with orientation."""
        # Center grid at origin (0,0)
        grid_marker = Marker()
        grid_marker.header.frame_id = "map"
        grid_marker.header.stamp = now
        grid_marker.ns = "reference"
        grid_marker.id = 3
        grid_marker.type = Marker.CUBE
        grid_marker.action = Marker.ADD
        grid_marker.pose.position.x = 0.0
        grid_marker.pose.position.y = 0.0
        grid_marker.pose.position.z = -0.1
        grid_marker.pose.orientation.w = 1.0
        grid_marker.scale.x = 20.0  # 20-meter reference grid
        grid_marker.scale.y = 20.0
        grid_marker.scale.z = 0.01
        grid_marker.color.r = 0.8
        grid_marker.color.g = 0.8
        grid_marker.color.b = 0.8
        grid_marker.color.a = 0.3
        marker_array.markers.append(grid_marker)
        
        # Add grid lines at 5-meter intervals
        for i in range(-10, 11, 5):
            if i == 0:
                continue  # Skip center
                
            # Horizontal line
            h_line = Marker()
            h_line.header.frame_id = "map"
            h_line.header.stamp = now
            h_line.ns = "reference"
            h_line.id = 100 + i
            h_line.type = Marker.LINE_STRIP
            h_line.action = Marker.ADD
            h_line.pose.orientation.w = 1.0
            h_line.scale.x = 0.1  # Line width
            h_line.color.r = 0.5
            h_line.color.g = 0.5
            h_line.color.b = 0.5
            h_line.color.a = 0.5
            
            p1 = Point()
            p1.x = -10.0
            p1.y = float(i)
            p1.z = 0.0
            
            p2 = Point()
            p2.x = 10.0
            p2.y = float(i)
            p2.z = 0.0
            
            h_line.points = [p1, p2]
            marker_array.markers.append(h_line)
            
            # Vertical line
            v_line = Marker()
            v_line.header.frame_id = "map"
            v_line.header.stamp = now
            v_line.ns = "reference"
            v_line.id = 200 + i
            v_line.type = Marker.LINE_STRIP
            v_line.action = Marker.ADD
            v_line.pose.orientation.w = 1.0
            v_line.scale.x = 0.1  # Line width
            v_line.color.r = 0.5
            v_line.color.g = 0.5
            v_line.color.b = 0.5
            v_line.color.a = 0.5
            
            p1 = Point()
            p1.x = float(i)
            p1.y = -10.0
            p1.z = 0.0
            
            p2 = Point()
            p2.x = float(i)
            p2.y = 10.0
            p2.z = 0.0
            
            v_line.points = [p1, p2]
            marker_array.markers.append(v_line)
        
        # Add a north arrow
        north_arrow = Marker()
        north_arrow.header.frame_id = "map"
        north_arrow.header.stamp = now
        north_arrow.ns = "reference"
        north_arrow.id = 4
        north_arrow.type = Marker.ARROW
        north_arrow.action = Marker.ADD
        north_arrow.pose.position.x = 0.0
        north_arrow.pose.position.y = 0.0
        north_arrow.pose.position.z = 0.1
        north_arrow.pose.orientation.w = 1.0
        north_arrow.scale.x = 5.0  # Arrow length
        north_arrow.scale.y = 0.5  # Arrow width
        north_arrow.scale.z = 0.5  # Arrow height
        north_arrow.color.r = 0.0
        north_arrow.color.g = 0.0
        north_arrow.color.b = 0.0
        north_arrow.color.a = 0.7
        marker_array.markers.append(north_arrow)

    def publish_foxglove_data(self):
        # Create a GeoJSON feature collection for Foxglove map panel
        geo_data = {
            "type": "FeatureCollection",
            "features": []
        }

        # Add raw path as a feature
        if self.raw_points:
            raw_path_feature = {
                "type": "Feature",
                "properties": {
                    "name": "Raw GPS Path",
                    "style": {
                        "color": "#ff0000",
                        "weight": 2,
                        "opacity": 0.7
                    }
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[p['lon'], p['lat']] for p in self.raw_points]
                }
            }
            geo_data["features"].append(raw_path_feature)

        # Add filtered path as a feature
        if self.filtered_points:
            filtered_path_feature = {
                "type": "Feature",
                "properties": {
                    "name": "Filtered GPS Path",
                    "style": {
                        "color": "#0000ff",
                        "weight": 3,
                        "opacity": 1.0
                    }
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [[p['lon'], p['lat']] for p in self.filtered_points]
                }
            }
            geo_data["features"].append(filtered_path_feature)

            # Add current position as a point feature
            current_pos_feature = {
                "type": "Feature",
                "properties": {
                    "name": "Current Position",
                    "style": {
                        "color": "#00ff00",
                        "radius": 5,
                        "opacity": 1.0,
                        "fillColor": "#00ff00",
                        "fillOpacity": 0.8
                    }
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [self.filtered_points[-1]['lon'], self.filtered_points[-1]['lat']]
                }
            }
            geo_data["features"].append(current_pos_feature)

        # Publish as a string message
        msg = String()
        msg.data = json.dumps(geo_data)
        self.foxglove_pub.publish(msg)

    def save_path_data(self):
        if not self.have_first_point:
            return

        # Create a data structure with all path information
        path_data = {
            "timestamp": datetime.now().isoformat(),
            "origin": {
                "latitude": self.origin_lat,
                "longitude": self.origin_lon
            },
            "raw_path": [{k: p[k] for k in p if k != 'viz_x' and k != 'viz_y'} for p in self.raw_points],
            "filtered_path": [{k: p[k] for k in p if k != 'viz_x' and k != 'viz_y'} for p in self.filtered_points]
        }

        # Save to file
        with open(self.log_file, 'w') as f:
            json.dump(path_data, f, indent=2)

        self.get_logger().info(f'Saved path data to: {self.log_file}')

def main():
    rclpy.init()
    node = FoxglovePathPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
