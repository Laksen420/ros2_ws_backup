#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json
import math
import pyproj
from pyproj import Transformer

class GeoJSONPublisherNode(Node):
    def __init__(self):
        super().__init__('geojson_publisher_node')
        
        # Create publisher for GeoJSON data
        self.geojson_pub = self.create_publisher(String, 'vehicle_path_geojson', 10)
        
        # Create subscription to path
        self.path_sub = self.create_subscription(
            Path,
            'vehicle_path',
            self.path_callback,
            10)
        
        # Subscribe to NavSatFix for UTM to LatLon conversion reference
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10)
        
        # Initialize variables
        self.latest_gps = None
        self.utm_zone = None
        self.utm_band = None
        self.transformer = None
        
        self.get_logger().info('GeoJSON publisher node initialized')
    
    def gps_callback(self, msg):
        # Store latest GPS data for reference
        self.latest_gps = msg
        
        # Determine UTM zone and band from the GPS coordinates
        if self.utm_zone is None:
            self.utm_zone = math.floor((msg.longitude + 180) / 6) + 1
            
            # Determine UTM band
            if msg.latitude >= 72.0:
                self.utm_band = 'X'
            elif msg.latitude >= 64.0:
                self.utm_band = 'W'
            elif msg.latitude >= 56.0:
                self.utm_band = 'V'
            elif msg.latitude >= 48.0:
                self.utm_band = 'U'
            elif msg.latitude >= 40.0:
                self.utm_band = 'T'
            elif msg.latitude >= 32.0:
                self.utm_band = 'S'
            elif msg.latitude >= 24.0:
                self.utm_band = 'R'
            elif msg.latitude >= 16.0:
                self.utm_band = 'Q'
            elif msg.latitude >= 8.0:
                self.utm_band = 'P'
            elif msg.latitude >= 0.0:
                self.utm_band = 'N'
            elif msg.latitude >= -8.0:
                self.utm_band = 'M'
            elif msg.latitude >= -16.0:
                self.utm_band = 'L'
            elif msg.latitude >= -24.0:
                self.utm_band = 'K'
            elif msg.latitude >= -32.0:
                self.utm_band = 'J'
            elif msg.latitude >= -40.0:
                self.utm_band = 'H'
            elif msg.latitude >= -48.0:
                self.utm_band = 'G'
            elif msg.latitude >= -56.0:
                self.utm_band = 'F'
            else:
                self.utm_band = 'C'
            
            # Set up transformer
            utm_crs = f"+proj=utm +zone={self.utm_zone} +{'' if msg.latitude >= 0 else 'south'} +datum=WGS84 +units=m +no_defs"
            self.transformer = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
            
            self.get_logger().info(f'UTM Zone/Band determined: {self.utm_zone}{self.utm_band}')
    
    def path_callback(self, path_msg):
        if not path_msg.poses or self.transformer is None:
            return
        
        # Create GeoJSON feature collection
        feature_collection = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "name": "Vehicle Path",
                        "color": "#0000FF"
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": []
                    }
                }
            ]
        }
        
        # Extract coordinates from path
        for pose in path_msg.poses:
            # Convert UTM coordinates to Lat/Lon
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            try:
                lon, lat = self.transformer.transform(x, y)
                feature_collection["features"][0]["geometry"]["coordinates"].append([lon, lat])
            except Exception as e:
                self.get_logger().error(f'Transformation error: {e}')
        
        # Convert to JSON string
        geojson_str = json.dumps(feature_collection)
        
        # Publish as string message
        msg = String()
        msg.data = geojson_str
        self.geojson_pub.publish(msg)

def main():
    rclpy.init()
    node = GeoJSONPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
