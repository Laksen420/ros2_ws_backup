#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from sensor_msgs.msg import NavSatFix
import requests
import json
from datetime import datetime

class AzureSenderNode(Node):
    def __init__(self):
        super().__init__('azure_sender_node')
        self.get_logger().info("AzureSenderNode starting...")

        # Subscribe to continuously published GPS data
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            'gps_data',
            self.gps_callback,
            10
        )
        # Subscribe to continuously published RFID best tag data
        self.rfid_subscriber = self.create_subscription(
            String,
            'rfid_best_tag',
            self.rfid_callback,
            10
        )
        # Subscribe to the trigger topic (from your trigger node)
        self.trigger_subscriber = self.create_subscription(
            Empty,
            'trigger',
            self.trigger_callback,
            10
        )

        # Data holders
        self.latest_gps = None
        self.latest_rfid = None

        # URL for your Azure Function HTTP trigger.
        # Replace <your_function_app> and <your_function_key> accordingly.
        self.azure_function_url = "https://gutenspass.azurewebsites.net/api/ReceiveGpsDataFunction?code=7_9W847HkKXDj_TAqrmg0I6bryOYFyv0UccWhyC42A1GAzFuWjvRdw%3D%3D"

    def gps_callback(self, msg: NavSatFix):
        self.latest_gps = msg
        self.get_logger().debug(
            f"GPS updated: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}"
        )

    def rfid_callback(self, msg: String):
        self.latest_rfid = msg.data
        self.get_logger().debug(f"RFID updated: {msg.data}")

    def trigger_callback(self, msg: Empty):
        self.get_logger().info("Trigger received in AzureSenderNode. Aggregating data...")
        if self.latest_gps is None:
            self.get_logger().warn("No valid GPS data available; aborting send.")
            return
        if self.latest_rfid is None:
            self.get_logger().warn("No RFID data available; aborting send.")
            return

        timestamp = datetime.utcnow().isoformat() + "Z"
        payload = {
            "timestamp": timestamp,
            "latitude": self.latest_gps.latitude,
            "longitude": self.latest_gps.longitude,
            "rfidId": self.latest_rfid,
            "rfidText": self.latest_rfid,  # You can expand this if needed
            "isPersonalPosition": False
        }
        self.get_logger().info(f"Sending payload: {payload}")

        try:
            response = requests.post(self.azure_function_url,
                                     json=payload,
                                     timeout=5)
            if response.status_code == 200:
                self.get_logger().info("Data sent to Azure successfully.")
            else:
                self.get_logger().error(f"Azure response: {response.status_code} - {response.text}")
        except Exception as e:
            self.get_logger().error(f"Error sending data to Azure: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AzureSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
