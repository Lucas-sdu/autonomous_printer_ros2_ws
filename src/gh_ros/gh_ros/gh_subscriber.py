#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GrasshopperSubscriber(Node):
    def __init__(self):
        super().__init__('grasshopper_subscriber')

        # Create a subscription to the '/grasshopper_input' topic
        self.subscription = self.create_subscription(
            String,
            '/grasshopper_input',
            self.listener_callback,
            10
        )

        # Create a publisher for the 'gcode_command' topic
        self.publisher_ = self.create_publisher(String, 'gcode_command', 10)

        self.get_logger().info('GrasshopperSubscriber node initialized.')

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'Received message: "{msg.data}"')

        # Republish the received G-code to the 'gcode_command' topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Republished to gcode_command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = GrasshopperSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GrasshopperSubscriber.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
