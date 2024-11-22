#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class GcodeManager(Node):
    def __init__(self):
        super().__init__('gcode_manager')

        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Default serial port
        self.declare_parameter('baud_rate', 115200)           # Default baud rate

        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = 2.0
        self.serial_conn = None

        # Initialize serial connection
        self.connect_to_printer()

    def connect_to_printer(self):
        if not self.serial_port:
            self.get_logger().error("The serial_port parameter is not set or is invalid.")
            return
        if self.baud_rate <= 0:
            self.get_logger().error("The baud_rate parameter must be a positive integer.")
            return

        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.timeout
            )
            self.get_logger().info(f"Connected to printer on {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_conn = None  # Ensure it remains None if connection fails

    def send_gcode(self, gcode):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not open.")
            return
        try:
            self.serial_conn.write((gcode + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent G-code: {gcode}")
            self.wait_for_ok()
        except Exception as e:
            self.get_logger().error(f"Failed to send G-code: {e}")

    def wait_for_ok(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not available.")
            return

        try:
            while True:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Printer Response: {response}")
                    if "ok" in response.lower():
                        self.get_logger().info("Received 'ok' from printer.")
                        break
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")

    def destroy(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gcode_manager = GcodeManager()

    # Test sending a simple G-code command
    gcode_manager.send_gcode('M105')  # Get current temperatures

    rclpy.spin_once(gcode_manager)
    gcode_manager.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
