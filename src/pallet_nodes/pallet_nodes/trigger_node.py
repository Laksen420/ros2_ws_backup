#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        self.get_logger().info("TriggerNode started. Press Enter to trigger a pallet event, or 'q' to quit.")

        # Create a publisher for /pallet_trigger
        self.trigger_pub = self.create_publisher(Bool, 'pallet_trigger', 10)

        # We’ll run a small loop in a separate thread or timer to check user input
        self.timer = self.create_timer(0.5, self.check_input)

    def check_input(self):
        """Check if user pressed Enter or typed 'q'."""
        # We’ll do a non-blocking read by using select or try...except approach
        import sys
        import select

        # If there's input ready on stdin, read it
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            line = sys.stdin.readline().strip()  # read one line
            if line.lower() == 'q':
                self.get_logger().info("Quitting TriggerNode...")
                # Shutdown the node gracefully
                rclpy.shutdown()
                return

            # If user just pressed Enter (line == ''), we publish True
            self.get_logger().info("Publishing a pallet trigger (Bool=True).")
            msg = Bool()
            msg.data = True
            self.trigger_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    rclpy.spin(node)  # spin will block until node is shut down

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
