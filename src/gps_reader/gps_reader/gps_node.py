#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import pynmea2
import time
import ssl
import json
import os

from sensor_msgs.msg import NavSatFix
from datetime import datetime

# paho-mqtt client
from paho.mqtt import client as mqtt

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.get_logger().info("GPSNode with PPP-RTK starting...")

        # -----------------------------
        # 1) Setup GNSS Serial Connection
        # -----------------------------
        self.serial_port = None
        self.open_gnss_serial_port()

        # Create a ROS 2 publisher for /gps_data
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Timer for reading data from serial port
        self.create_timer(0.01, self.read_serial_data)

        # -----------------------------
        # 2) Setup Certificate Paths
        # -----------------------------
        # Determine the directory of the current script
        current_dir = os.path.dirname(os.path.realpath(__file__))

        # Define the 'certs' directory path relative to the script
        certs_dir = os.path.join(current_dir, "certs")

        # Define the paths to the certificate and key files
        self.ca_certs = "/etc/ssl/certs/ca-certificates.crt"  # Default CA certs
        self.certfile = "/home/ubuntu/ros2_ws/src/gps_reader/gps_reader/certs/device-88653537-0ec2-4a17-ab28-c4ecde3345e2-pp-cert.crt"
        self.keyfile  = "/home/ubuntu/ros2_ws/src/gps_reader/gps_reader/certs/device-88653537-0ec2-4a17-ab28-c4ecde3345e2-pp-key.pem"

        self.get_logger().info(f"Certificate file path: {self.certfile}")
        self.get_logger().info(f"Key file path: {self.keyfile}")


        # Debugging: Log the paths being used
        self.get_logger().debug(f"CA Certs Path: {self.ca_certs}")
        self.get_logger().debug(f"Certificate Path: {self.certfile}")
        self.get_logger().debug(f"Key Path: {self.keyfile}")

        # Verify certificate files exist
        if not (os.path.exists(self.certfile) and os.path.exists(self.keyfile)):
            self.get_logger().error("Certificate or key file not found. Please check the paths.")
            self.get_logger().error(f"Looking for certfile at: {self.certfile}")
            self.get_logger().error(f"Looking for keyfile at: {self.keyfile}")
            return
        else:
            self.get_logger().info("Certificate and key files found.")

        # -----------------------------
        # 3) Setup PointPerfect MQTT
        # -----------------------------
        self.pp_client_id = "laksen"  # Replace with your actual PP Client ID
        self.pp_broker_address = "pp.services.u-blox.com"
        self.pp_broker_port = 8883

        # Create paho MQTT client for PPP
        self.pp_client = mqtt.Client(client_id=self.pp_client_id)
        self.pp_client.on_connect = self.on_connect_pp
        self.pp_client.on_message = self.on_message_pp

        # Setup TLS for PPP
        try:
            self.pp_client.tls_set(
                ca_certs=self.ca_certs,
                certfile=self.certfile,
                keyfile=self.keyfile,
                cert_reqs=ssl.CERT_REQUIRED,
                tls_version=ssl.PROTOCOL_TLSv1_2
            )
            self.get_logger().info("TLS configuration successful.")
        except ssl.SSLError as e:
            self.get_logger().error(f"SSL configuration error: {e}")
            return

        self.pp_client.tls_insecure_set(False)

        # Connect and start the MQTT loop in a background thread
        try:
            self.pp_client.connect(self.pp_broker_address, self.pp_broker_port, keepalive=60)
            self.pp_client.loop_start()
            self.get_logger().info("Connecting to PointPerfect MQTT Broker...")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to PPP broker: {e}")

    # -----------------------------
    # GNSS Serial Port Handling
    # -----------------------------
    def open_gnss_serial_port(self):
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
            self.get_logger().info("Opened GNSS serial port successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

    def read_serial_data(self):
        """Periodic callback to read from the GNSS device and publish NavSatFix."""
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if data:
                # For simplicity, split lines on '\n'
                lines = data.decode('ascii', errors='ignore').split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith('$'):
                        self.process_nmea_sentence(line)
        except Exception as e:
            self.get_logger().error(f"Error reading GNSS serial: {e}")

    def process_nmea_sentence(self, sentence):
        try:
            msg = pynmea2.parse(sentence)
            if isinstance(msg, pynmea2.GGA):
                # Build and publish a NavSatFix
                navsat_msg = NavSatFix()
                navsat_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_msg.header.frame_id = "gps_link"

                navsat_msg.latitude = msg.latitude
                navsat_msg.longitude = msg.longitude
                navsat_msg.altitude = float(msg.altitude) if msg.altitude else 0.0
                # Optionally parse HDOP, # sats, etc.

                self.publisher_.publish(navsat_msg)
                self.get_logger().info(
                    f"PPP-GPS lat={navsat_msg.latitude:.6f}, "
                    f"lon={navsat_msg.longitude:.6f}, alt={navsat_msg.altitude:.2f}"
                )
        except pynmea2.ParseError as e:
            self.get_logger().warn(f"NMEA parse error: {e}")

    # -----------------------------
    # PointPerfect MQTT Callbacks
    # -----------------------------
    def on_connect_pp(self, client, userdata, flags, rc):
        """Callback when MQTT client connects to PPP broker."""
        if rc == 0:
            self.get_logger().info("Connected to PointPerfect MQTT Broker!")
            # Subscribe to SPARTN key topic
            key_topic = "/pp/ubx/0236/ip"
            self.get_logger().info(f"Subscribing to key topic: {key_topic}")
            client.subscribe((key_topic, 1))

            # Subscribe to AssistNow (if needed)
            assistnow_topic = "/pp/ubx/mga"
            self.get_logger().info(f"Subscribing to AssistNow topic: {assistnow_topic}")
            client.subscribe((assistnow_topic, 1))

            # Subscribe to regional correction (EU, NA, etc.) or dynamic tile
            spartn_topic = "/pp/ip/eu"  # example: Europe region
            self.get_logger().info(f"Subscribing to correction topic: {spartn_topic}")
            client.subscribe((spartn_topic, 0))
        else:
            self.get_logger().error(f"Failed to connect PPP, rc={rc}")

    def on_message_pp(self, client, userdata, msg):
        """Handle incoming PPP correction data or keys from the broker."""
        topic = msg.topic
        payload = msg.payload

        # If it's a SPARTN key or AssistNow data, send to GNSS
        if topic.startswith("/pp/ubx/"):
            self.write_to_gnss_device(payload)
            self.get_logger().info("Sent SPARTN key/AssistNow data to GNSS")
        # If it's SPARTN correction data, also send to GNSS
        elif topic.startswith("/pp/ip/"):
            self.write_to_gnss_device(payload)
            self.get_logger().info("Sent SPARTN correction data to GNSS")
        else:
            self.get_logger().warn(f"Received message on unexpected topic: {topic}")

    def write_to_gnss_device(self, payload):
        """Write the correction data or keys to the GNSS device over serial."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(payload)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send data to GNSS: {e}")
        else:
            self.get_logger().error("Serial port not open; cannot send corrections.")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
