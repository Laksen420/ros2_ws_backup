#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json
import os
from datetime import datetime

class FoxgloveMapPublisherNode(Node):
    def __init__(self):
        super().__init__('foxglove_map_publisher')
        
        # The Map panel accepts NavSatFix messages directly, so we republish them
        # We also create a GeoJSON publisher for the path visualization
        self.geojson_pub = self.create_publisher(String, '/foxglove/geojson', 10)
        
        # Subscribe to GPS data - both original and filtered
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10)
            
        self.filtered_gps_sub = self.create_subscription(
            NavSatFix,
            'gps/filtered',
            self.filtered_gps_callback,
            10)
        
        # Store path data
        self.path_points = []
        
        # Path saving parameters
        self.declare_parameter('save_path', True)
        self.declare_parameter('save_interval', 5.0)  # seconds
        
        self.save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # Create log directory if saving is enabled
        if self.save_path:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_dir = os.path.join(os.getcwd(), 'path_logs')
            os.makedirs(self.log_dir, exist_ok=True)
            self.log_file = os.path.join(self.log_dir, f'gps_path_{timestamp}.json')
            
            # Initialize save timer
            self.save_timer = self.create_timer(self.save_interval, self.save_path_data)
            
            self.get_logger().info(f'Path data will be saved to: {self.log_file}')
        
        # Create a timer for publishing the GeoJSON path
        self.geojson_timer = self.create_timer(0.5, self.publish_geojson)
        
        self.get_logger().info('Foxglove Map Publisher initialized')
    
    def gps_callback(self, msg):
        # Store data points for the path - we'll use original GPS for path
        point = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "type": "raw"
        }
        self.path_points.append(point)
        
        # Limit the path length to prevent memory issues
        if len(self.path_points) > 1000:
            self.path_points = self.path_points[-1000:]
    
    def filtered_gps_callback(self, msg):
        # Store filtered GPS points for the path
        point = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "type": "filtered"
        }
        self.path_points.append(point)
        
        # Limit the path length to prevent memory issues
        if len(self.path_points) > 1000:
            self.path_points = self.path_points[-1000:]
    
    def publish_geojson(self):
        if not self.path_points:
            return
        
        # Create GeoJSON feature collection according to Foxglove docs
        feature_collection = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "name": "Vehicle Path",
                        "style": {
                            "color": "#ff0000",  # Red color for path
                            "weight": 3,         # Line width
                            "opacity": 1
                        }
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": []
                    }
                }
            ]
        }
        
        # Add the filtered points to the path
        filtered_points = [p for p in self.path_points if p["type"] == "filtered"]
        
        # If no filtered points, use raw points
        if not filtered_points:
            filtered_points = [p for p in self.path_points if p["type"] == "raw"]
        
        # Sort by timestamp to ensure correct path order
        filtered_points.sort(key=lambda p: p["timestamp"])
        
        # Add points to LineString
        for point in filtered_points:
            feature_collection["features"][0]["geometry"]["coordinates"].append(
                [point["longitude"], point["latitude"], point["altitude"]]
            )
        
        # Add a point marker for the current position
        if filtered_points:
            current_point = filtered_points[-1]
            feature_collection["features"].append({
                "type": "Feature",
                "properties": {
                    "name": "Current Position",
                    "style": {
                        "color": "#0000ff",  # Blue for current position
                        "weight": 5,         # Point size
                        "opacity": 1,
                        "fillColor": "#0000ff",
                        "fillOpacity": 0.8
                    }
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [current_point["longitude"], current_point["latitude"], current_point["altitude"]]
                }
            })
        
        # Publish GeoJSON as a string message
        msg = String()
        msg.data = json.dumps(feature_collection)
        self.geojson_pub.publish(msg)
    
    def save_path_data(self):
        if not self.path_points:
            return
        
        # Create a GeoJSON object for saving that includes timestamps
        geojson = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "name": "Recorded Path",
                        "timestamp": datetime.now().isoformat(),
                        "points": self.path_points
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": []
                    }
                }
            ]
        }
        
        # Add the coordinates to the LineString
        for point in sorted(self.path_points, key=lambda p: p["timestamp"]):
            geojson["features"][0]["geometry"]["coordinates"].append(
                [point["longitude"], point["latitude"], point["altitude"]]
            )
        
        # Save to file
        with open(self.log_file, 'w') as f:
            json.dump(geojson, f, indent=2)
            
        self.get_logger().info(f'Saved path data to: {self.log_file}')

def main():
    rclpy.init()
    node = FoxgloveMapPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
