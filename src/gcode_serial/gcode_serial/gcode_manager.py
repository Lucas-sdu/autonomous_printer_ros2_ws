#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from collections import deque
from std_msgs.msg import String

class GcodeManager(Node):
    def __init__(self):
        super().__init__('gcode_manager')

        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Default serial port
        self.declare_parameter('baud_rate', 115200)           # Default baud rate
        
        self.printer_status_pub = self.create_publisher(String, '/printer_status', 10)
        self.get_logger().info("GCodeManager Node Initialized.")
        # Get parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = 2.0
        self.serial_conn = None

        # Initialize serial connection and wait for startup
        self.connect_to_printer()

        # Initialize G-code command buffer
        self.command_buffer = deque()
        self.processing_command = False
        self.last_command_time = None

        # Subscribe to /grasshopper_input topic
        self.subscription = self.create_subscription(
            String,
            '/grasshopper_input',
            self.grasshopper_input_callback,
            10
        )
        self.get_logger().info("Subscribed to /grasshopper_input topic.")

        # Create a timer to periodically check the buffer and send commands
        self.command_timer = self.create_timer(0.5, self.process_command_buffer)

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

            # Wait for the printer to send all startup messages
            self.wait_for_printer_ready()

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_conn = None  # Ensure it remains None if connection fails

    def wait_for_printer_ready(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not available or open.")
            return

        self.get_logger().info("Waiting for printer to finish startup...")

        # Wait for 7 seconds and log all responses
        start_time = time.time()
        while time.time() - start_time < 7.0:
            try:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Printer Response: {response}")
                    self.publish_printer_status(response)

                    # If the response contains "busy", prompt for user input
                    if "busy" in response.lower():
                        self.get_logger().warning("Printer is busy. Please resolve the issue (e.g., press a button) and press Enter to continue...")
                        input("Press Enter to continue once the printer is ready...")
                        start_time = time.time()  # Reset the timeout
                        continue

                    # If a critical error occurs, log it
                    if "error" in response.lower():
                        self.get_logger().error(f"Printer Error: {response}")
                        break
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial connection: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error while waiting for printer startup: {e}")
                break

        # Add manual confirmation from user
        input("Manually confirm that the printer is ready. Press Enter to continue...")
        self.get_logger().info("Startup wait complete. Printer should be ready now.")

    def grasshopper_input_callback(self, msg):
        gcode_command = msg.data.strip()
        if gcode_command:
            self.add_gcode_to_buffer(gcode_command)
            self.last_command_time = time.time()

    def add_gcode_to_buffer(self, gcode):
        self.command_buffer.append(gcode)
        self.get_logger().info(f"Added G-code to buffer: {gcode}")
        #this lines is for debugging the mighty buffer
       #self.display_buffer()

    def process_command_buffer(self):
        if self.last_command_time is not None:
            time_since_last_command = time.time() - self.last_command_time
            if time_since_last_command < 0.4:
                # Wait for at least 1 second before starting to send commands
                return

        if not self.processing_command and self.command_buffer and self.serial_conn and self.serial_conn.is_open:
            self.send_next_command()

    def send_next_command(self):
        if self.command_buffer and self.serial_conn and self.serial_conn.is_open:
            next_gcode = self.command_buffer.popleft()
            self.get_logger().info(f"Sending next G-code from buffer: {next_gcode}")
            #self.display_buffer()
            self.processing_command = True
            self.send_gcode(next_gcode)

    def send_gcode(self, gcode):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not open.")
            return
        try:
            # Record the start time
            start_time = time.time()

            self.get_logger().info(f"Attempting to send G-code: {gcode}")
            self.serial_conn.write((gcode + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent G-code: {gcode}")

            # Pass start_time to wait_for_ok
            self.wait_for_ok(start_time)

            # Check for additional responses after the initial "ok"
            self.check_for_additional_responses()
        except Exception as e:
            self.get_logger().error(f"Failed to send G-code: {e}")

    def wait_for_ok(self, start_time):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not available.")
            return

        try:
            while True:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Printer Response: {response}")
                    self.publish_printer_status(response)
                    if "ok" in response.lower():
                        # Record the end time and calculate elapsed time
                        end_time = time.time()
                        elapsed_time = end_time - start_time
                        self.get_logger().info(f"Elapsed time: {elapsed_time:.3f} seconds")

                        self.get_logger().info("Received 'ok' from printer. Command completed successfully.")
                        self.processing_command = False

                        break
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")

    def check_for_additional_responses(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection is not available.")
            return

        try:
            self.get_logger().info("Checking for additional responses from the printer...")
            while True:
                response = self.serial_conn.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Additional Printer Response: {response}")
                    self.publish_printer_status(response)
                else:
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading additional responses from serial: {e}")

    def display_buffer(self):
        self.get_logger().info(f"Current Buffer: {list(self.command_buffer)}")

    def publish_printer_status(self, message):
        msg = String()
        msg.data = message
        self.printer_status_pub.publish(msg)

    def destroy(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gcode_manager = GcodeManager()
    time.sleep(2)
    
    rclpy.spin(gcode_manager)
    
    gcode_manager.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
