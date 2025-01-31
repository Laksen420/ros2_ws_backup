#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json

# If using standard messages for GPS or RFID, import them, e.g.:
from sensor_msgs.msg import NavSatFix
# For RFID, you may have a custom message or standard type; adjust accordingly

class AzureBridgeNode(Node):
    def __init__(self):
        super().__init__('azure_bridge_node')
        self.get_logger().info("AzureBridgeNode starting...")

        # -----------------------------------------
        # 1) Config: Replace with your actual endpoint & logic
        # -----------------------------------------
        self.azure_function_url = "https://functionpsykose.azurewebsites.net/api/ReceiveGpsDataFunction?code=3HSq_sMf7AqlL0qH7VveV5IFyUpZpcSoQ84I6USsgSDmAzFucW3VYA%3D%3D"
        # If using SignalR or a different endpoint, update accordingly

        # Optional: If you need an API key or token:
        self.headers = {
            "Content-Type": "application/json",
            # "Authorization": "Bearer <YOUR_TOKEN>"
        }

        # -----------------------------------------
        # 2) Subscriptions
        # -----------------------------------------
        # Example: subscribe to /gps_data for NavSatFix
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )

        # If you have an RFID topic, subscribe similarly:
        # self.rfid_sub = self.create_subscription(
        #     RfidMsg,  # Replace with your actual msg type
        #     'rfid_data',
        #     self.rfid_callback,
        #     10
        # )

    def gps_callback(self, msg):
        """Handle incoming GPS data and send to Azure."""
        lat = msg.latitude
        lon = msg.longitude
        # ... alt = msg.altitude, etc.

        data_payload = {
            "timestamp": self.get_clock().now().to_msg().sec,
            "latitude": lat,
            "longitude": lon,
            "rfidId": None,  # or "12345" if you combine data from RFID
            "rfidText": "From AzureBridgeNode",
            "ageDiffCorr": 0.0
        }

        self.send_to_azure(data_payload)

    # def rfid_callback(self, msg):
    #     """Handle incoming RFID data and possibly send to Azure."""
    #     # Example structure:
    #     rfid_id = msg.idHex
    #     rssi = msg.rssi
    #
    #     data_payload = {
    #         "timestamp": self.get_clock().now().to_msg().sec,
    #         "latitude": 0.0,  # or some stored GPS value if you want to combine
    #         "longitude": 0.0,
    #         "rfidId": rfid_id,
    #         "rfidText": "From AzureBridgeNode",
    #         "ageDiffCorr": 0.0
    #     }
    #
    #     self.send_to_azure(data_payload)

    def send_to_azure(self, payload):
        """Sends data to your Azure function or SignalR endpoint."""
        try:
            response = requests.post(self.azure_function_url,
                                     headers=self.headers,
                                     json=payload)
            response.raise_for_status()
            self.get_logger().info(f"Sent to Azure: {payload}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send data to Azure: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AzureBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
