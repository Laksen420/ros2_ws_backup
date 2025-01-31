#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
import json
from datetime import datetime

class PalletFusionNode(Node):
    def __init__(self):
        super().__init__('pallet_fusion_node')
        self.get_logger().info("PalletFusionNode started.")

        # Subscribe to trigger
        self.trigger_sub = self.create_subscription(
            Bool,
            'pallet_trigger',
            self.trigger_callback,
            10
        )

        # Subscribe to RTK/GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )

        # Subscribe to RFID
        self.rfid_sub = self.create_subscription(
            String,
            'rfid_data',
            self.rfid_callback,
            10
        )

        # Publisher for the final "pallet event"
        self.pallet_event_pub = self.create_publisher(String, 'pallet_event', 10)

        # Local caches
        self.latest_lat = None
        self.latest_lon = None
        self.latest_rfid_epc = None

    def gps_callback(self, msg):
        # Cache the latest lat/lon
        self.latest_lat = msg.latitude
        self.latest_lon = msg.longitude
        # self.get_logger().info(f"Caching GPS: lat={self.latest_lat}, lon={self.latest_lon}")

    def rfid_callback(self, msg):
        # Cache the latest RFID EPC string
        self.latest_rfid_epc = msg.data
        # self.get_logger().info(f"Caching RFID EPC: {self.latest_rfid_epc}")

    def trigger_callback(self, msg):
        if msg.data:
            # We only care if Bool=True
            self.get_logger().info("Received a Trigger => producing a pallet event...")

            # Build the JSON structure
            # If we don't have lat/lon or rfid, fallback to something
            lat = self.latest_lat if self.latest_lat is not None else 0.0
            lon = self.latest_lon if self.latest_lon is not None else 0.0
            rfid = self.latest_rfid_epc if self.latest_rfid_epc else "UNKNOWN"

            event_time = datetime.now().isoformat()

            # Create a dictionary
            event_dict = {
                "timestamp": event_time,
                "latitude": lat,
                "longitude": lon,
                "rfidEpc": rfid
            }
            # Convert to JSON string
            event_str = json.dumps(event_dict)

            # Publish on /pallet_event
            msg_out = String()
            msg_out.data = event_str
            self.pallet_event_pub.publish(msg_out)
            self.get_logger().info(f"PalletEvent published: {event_str}")

def main(args=None):
    rclpy.init(args=args)
    node = PalletFusionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
