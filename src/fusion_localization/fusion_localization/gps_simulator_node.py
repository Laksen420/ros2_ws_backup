#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import time
import random
from datetime import datetime

class GPSSimulatorNode(Node):
    def __init__(self):
        super().__init__('gps_simulator_node')
        
        # Create publisher for GPS data
        self.gps_pub = self.create_publisher(NavSatFix, 'gps_data', 10)
        
        # Parameters
        self.declare_parameter('center_lat', 59.9127)  # Oslo
        self.declare_parameter('center_lon', 10.7461)
        self.declare_parameter('radius', 10.0)         # Circle radius in meters
        self.declare_parameter('speed', 1.0)           # Movement speed in m/s
        self.declare_parameter('freq', 0.5)            # Update frequency in Hz (reduced to 0.5Hz)
        self.declare_parameter('noise_level', 0.2)     # GPS noise level (meters)
        self.declare_parameter('path_type', 'figure_eight')  # 'circle', 'figure_eight', 'random_walk'
        
        # Get parameters
        self.center_lat = self.get_parameter('center_lat').value
        self.center_lon = self.get_parameter('center_lon').value
        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        self.freq = self.get_parameter('freq').value
        self.noise_level = self.get_parameter('noise_level').value
        self.path_type = self.get_parameter('path_type').value
        
        # Variables for motion
        self.angle = 0.0
        self.start_time = time.time()
        
        # For random walk path
        self.current_lat = self.center_lat
        self.current_lon = self.center_lon
        self.heading = 0.0  # degrees
        
        # Create timer for publishing updates
        self.timer = self.create_timer(1.0/self.freq, self.publish_gps)
        
        self.get_logger().info(f'GPS Simulator started, publishing to "gps_data" topic')
        self.get_logger().info(f'Path type: {self.path_type}')
        
    def publish_gps(self):
        """Generate and publish simulated GPS data according to selected path type."""
        # Calculate elapsed time
        elapsed = time.time() - self.start_time
        
        # Generate position based on selected path type
        if self.path_type == 'circle':
            lat, lon = self.generate_circle_path(elapsed)
        elif self.path_type == 'figure_eight':
            lat, lon = self.generate_figure_eight_path(elapsed)
        elif self.path_type == 'random_walk':
            lat, lon = self.generate_random_walk()
        else:
            # Default to circle if invalid path type
            lat, lon = self.generate_circle_path(elapsed)
        
        # Add some noise to simulate real GPS behavior
        lat += self.meters_to_lat(random.uniform(-self.noise_level, self.noise_level))
        lon += self.meters_to_lon(random.uniform(-self.noise_level, self.noise_level), lat)

        self.get_logger().info(f'Path type: {self.path_type}, Coords: ({lat:.6f}, {lon:.6f})')
        # Create GPS message
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.latitude = lat
        msg.longitude = lon
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
        self.get_logger().info(f'Publishing simulated GPS: {msg.latitude:.8f}, {msg.longitude:.8f}')
    
    def generate_circle_path(self, elapsed):
        """Generate a circular path around the center coordinates."""
        # Simple constant angular velocity motion
        angular_velocity = self.speed / self.radius  # rad/s
        self.angle = (elapsed * angular_velocity) % (2.0 * math.pi)
        
        # Calculate position on circle
        lat_offset = self.meters_to_lat(self.radius * math.sin(self.angle))
        lon_offset = self.meters_to_lon(self.radius * math.cos(self.angle), self.center_lat)
        
        return self.center_lat + lat_offset, self.center_lon + lon_offset
        
        def generate_figure_eight_path(self, elapsed):
            """Generate a figure-eight path for more interesting movement."""
            # Calculate the angle based on elapsed time
            angular_velocity = self.speed / self.radius  # rad/s
            self.angle = (elapsed * angular_velocity) % (2.0 * math.pi)
            
            # Better figure eight formula using lemniscate of Bernoulli
            # This creates a true mathematical figure-eight shape
            denominator = 1.0 + math.sin(self.angle) * math.sin(self.angle)
            if denominator > 0:
                r = self.radius * math.cos(self.angle) / denominator
                
                # Convert polar to cartesian
                x = r * math.cos(self.angle)
                y = r * math.sin(self.angle)
            else:
                # Avoid division by zero
                x = 0.0
                y = 0.0
            
            # Convert to lat/lon offsets
            lat_offset = self.meters_to_lat(y)
            lon_offset = self.meters_to_lon(x, self.center_lat)
            
            self.get_logger().debug(f"Figure eight: angle={self.angle:.2f}, x={x:.2f}, y={y:.2f}")
            
            return self.center_lat + lat_offset, self.center_lon + lon_offset

    def generate_random_walk(self):
        """Generate a random walk path with smooth turns."""
        # Update heading with a small random change
        self.heading += random.uniform(-10, 10)  # Turn up to 10 degrees each step
        
        # Calculate new position based on heading and speed
        distance = self.speed / self.freq  # distance traveled in this step
        
        # Convert heading to radians
        heading_rad = math.radians(self.heading)
        
        # Calculate offsets
        dx = distance * math.sin(heading_rad)
        dy = distance * math.cos(heading_rad)
        
        # Convert to lat/lon offsets
        lat_offset = self.meters_to_lat(dy)
        lon_offset = self.meters_to_lon(dx, self.current_lat)
        
        # Update current position
        self.current_lat += lat_offset
        self.current_lon += lon_offset
        
        # If we're getting too far from center, steer back
        center_distance = self.haversine(self.center_lat, self.center_lon, 
                                         self.current_lat, self.current_lon)
        if center_distance > self.radius * 2:
            # Calculate bearing to center
            bearing = self.calculate_bearing(self.current_lat, self.current_lon,
                                            self.center_lat, self.center_lon)
            # Adjust heading to steer back
            self.heading = self.heading * 0.8 + bearing * 0.2
        
        return self.current_lat, self.current_lon
    
    def haversine(self, lat1, lon1, lat2, lon2):
        """Calculate great-circle distance between two points in meters."""
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # Earth radius in meters
        return c * r
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate initial bearing between two points in degrees."""
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Calculate bearing
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        # Convert to degrees
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        return bearing
    
    def meters_to_lat(self, meters):
        """Convert meters to latitude degrees."""
        return meters / 111320.0
    
    def meters_to_lon(self, meters, lat):
        """Convert meters to longitude degrees at the given latitude."""
        return meters / (111320.0 * math.cos(math.radians(lat)))

def main():
    rclpy.init()
    node = GPSSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
