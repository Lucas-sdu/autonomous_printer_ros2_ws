#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from datetime import datetime  # Import datetime

class ImageManagerNode(Node):
    def __init__(self):
        super().__init__('image_manager')

        # Publisher for /image_trigger (sending commands)
        self.trigger_publisher = self.create_publisher(String, '/image_trigger', 10)
        self.get_logger().info("Publishing trigger commands to /image_trigger.")

        # Subscriber for /image_input (receiving images)
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_input',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /image_input for receiving images.")

    def image_callback(self, msg):
        """Callback for handling incoming images on /image_input."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # Get current time
        self.get_logger().info(f"[{timestamp}] Received image on /image_input with format: {msg.format}")
def main(args=None):
    rclpy.init(args=args)
    node = ImageManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
