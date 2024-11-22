#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class GCodeSenderNode(Node):
    def __init__(self):
        super().__init__('gcode_sender')

        # Declare and get parameters for serial configuration
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize serial connection
        try:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {serial_port} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e

        # Subscribe to G-code commands
        self.subscription = self.create_subscription(
            String,
            'gcode_command',
            self.send_gcode_callback,
            10
        )
        self.get_logger().info('GCodeSenderNode is ready and listening for commands.')

    def send_gcode_callback(self, msg):
        gcode = msg.data
        self.get_logger().info(f"Sending G-code: {gcode}")
        try:
            # Send G-code command via serial
            self.serial_connection.write((gcode + '\n').encode())
            self.get_logger().info("G-code sent successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to send G-code: {e}")

    def destroy_node(self):
        # Close the serial connection when the node is shut down
        if self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GCodeSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GCodeSenderNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
