#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('arduino_comm_node')
        
        # Declare and get parameters for serial port and baud rate
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Serial setup
        try:
            self.arduino = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Give time for Arduino to reset
            self.get_logger().info('Connected to Arduino on {}'.format(serial_port))
        except serial.SerialException as e:
            self.get_logger().error('Could not open serial port: {}'.format(e))
        
        # Timer to regularly check and read data from Arduino
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.read_from_arduino)

    def read_from_arduino(self):
        # Flush any stale data to get only the latest value
        while self.arduino.in_waiting > 0:
            data = self.arduino.readline().decode('utf-8').strip()  # Read one line from the buffer
        self.get_logger().info(f'Received: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        if hasattr(node, 'arduino') and node.arduino.is_open:
            node.arduino.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
