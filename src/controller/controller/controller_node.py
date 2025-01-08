#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import re
from .states.startup_state import StartupState
from .states.scan_state import ScanState
from .states.placeholder_state import State
from .states.print_state import PrintState

from sensor_msgs.msg import CompressedImage

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Initialized")

        # Shared message store
        self.shared_data = {"grasshopper_input": None, "printer_status": None, "image_manager_input": None}
        
        ## Counter for print cycles
        self.print_cycles = 0  

        # Check Grasshopper connection
        self.connected_to_grasshopper = False

        # Required nodes for startup
        self.required_nodes = ["gcode_manager", "ros_to_gh", "gh_subscriber", "rosbridge_websocket", "image_manager"]

        # Current State
        self.current_state = None
        self.is_paused = False

        # User Command Subscriber
        self.command_sub = self.create_subscription(
            String, '/user_command', self.command_callback, 10)

        # Status Publisher
        self.status_pub = self.create_publisher(
            String, '/controller_status', 10)

        # Log Publisher
        self.log_pub = self.create_publisher(
            String, '/controller_log', 10)

        
        # State Publisher
        self.state_pub = self.create_publisher(String, '/controller_state', 10)

        # Stage Publisher
        self.stage_pub = self.create_publisher(String, '/controller_stage', 10)
        self.publish_none_stage()  # Publish NONE stage only at startup

        # Print Cycle publisher
        self.print_cycles_pub = self.create_publisher(String, '/print_cycles', 10)

        # Timer to continuously publish status and state
        self.timer = self.create_timer(1.0, self.publish_status)

        # Add the subscriber to listen for Grasshopper messages
        self.grasshopper_sub = self.create_subscription(
            String,'/grasshopper_input',self.grasshopper_callback,10)

        self.printer_status_sub = self.create_subscription(
            String, '/printer_status', self.printer_status_callback, 10)

        self.image_manager_sub = self.create_subscription(
            CompressedImage, '/image_input', self.image_manager_callback, 10)


        # Start the input listener thread
        self.listen_for_input()

    def image_manager_callback(self, msg):
        """Callback to handle received images from /image_input."""
        self.shared_data["image_input"] = True  # Just mark as received
        self.get_logger().info("Image received on /image_input.")  # Log receipt

    def grasshopper_callback(self, msg):
        """Store messages from Grasshopper in shared data"""
        self.shared_data["grasshopper_input"] = msg.data
        self.get_logger().info(f"Received message on '/grasshopper_input': {msg.data}")

    def publish_print_cycles(self):
        """Publish the current print cycle count."""
        print_cycles_msg = String()
        print_cycles_msg.data = str(self.print_cycles)
        self.print_cycles_pub.publish(print_cycles_msg)
        self.get_logger().info(f"Published Print Cycles: {self.print_cycles}")

    def printer_status_callback(self, msg):
        self.shared_data["printer_status"] = msg.data

        # Parse M114 response format
        if "Count" in msg.data:
            try:
                # Extract the portion after "Count"
                count_section = msg.data.split("Count")[-1].strip()

                # Use regex to extract numerical values for X, Y, Z
                match = re.search(r"X:\s*(-?\d+(\.\d+)?)\s+Y:(-?\d+(\.\d+)?)\s+Z:(-?\d+(\.\d+)?)", count_section)
                if match:
                    position = {
                        "X": float(match.group(1)),  # Group 1 matches X value
                        "Y": float(match.group(3)),  # Group 3 matches Y value
                        "Z": float(match.group(5)),  # Group 5 matches Z value
                    }
                    self.shared_data["last_actual_position"] = position
                    self.get_logger().info(f"Updated last_actual_position: {position}")
                else:
                    self.get_logger().warning(f"No valid position found in Count section: {count_section}")
            except Exception as e:
                self.get_logger().warning(f"Failed to parse M114 response: {msg.data} ({e})")


    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")

        if command == 'start':
            self.start()
        elif command == 'pause':
            self.pause()
        elif command.startswith('goto '):
            target_state_name = command.split(' ', 1)[1].upper()
            self.goto_state(target_state_name)
        elif command.startswith("SET_CYCLE:"):
            try:
                # Extract the value after "SET_CYCLE:"
                new_cycle_count = int(command.split(":")[1].strip())
                self.print_cycles = new_cycle_count
                self.publish_print_cycles()
                self.get_logger().info(f"Print cycles updated via user command to: {self.print_cycles}")
            except (ValueError, IndexError):
                self.get_logger().warning(f"Invalid SET_CYCLE command: {command}")
        
        else:
            self.get_logger().info(f"Unknown command: {command}")

    def publish_none_stage(self):
        """Publish 'State: NONE | Stage: NONE' to indicate initial startup."""
        stage_msg = String()
        stage_msg.data = "NONE"
        self.stage_pub.publish(stage_msg)
        self.get_logger().info("Published Initial Stage: NONE")

    def start(self):
        if self.current_state is None:
            # Start in StartupState
            self.publish_none_stage()  # Publish NONE stage only at startup
            self.current_state = StartupState("STARTUP_STATE", self.required_nodes, self.get_logger(), self, self.stage_pub)

            self.current_state.on_enter()
        self.is_paused = False
        self.get_logger().info("State machine started")

    def pause(self):
        if self.current_state is not None:
            self.is_paused = True
            self.get_logger().info("State machine paused")

    def publish_status(self):
        status_msg = String()
        log_msg = String()
        state_msg = String()  # For current state name

        active_nodes = self.get_node_names()  # Retrieve active nodes in the graph

        if self.current_state is None:
            status_msg.data = "State: NONE | Stage: NONE | Status: IDLE"
            log_msg.data = "LOG: No active state."
            state_msg.data = "NONE"
        else:
            # Retrieve the stage from the current state
            stage = getattr(self.current_state, 'stage', "N/A")

            if self.is_paused:
                status_msg.data = f"State: {self.current_state.name} | Stage: {stage} | Status: PAUSED"
                log_msg.data = f"LOG: {self.current_state.name} paused."
            else:
                status_msg.data = f"State: {self.current_state.name} | Stage: {stage} | Status: RUNNING"
                result = self.current_state.execute()

                if result == "done":
                    self.current_state.on_exit()
                    self.current_state = ScanState("SCAN_STATE", self.required_nodes, self.get_logger(), self, self.stage_pub)
                    self.current_state.on_enter()
                    self.print_cycles = 0 # Reset print cycle count
                    self.publish_print_cycles()  # Publish updated count
                    self.get_logger().info("Transitioning to SCAN_STATE.")

                elif result == "SCAN_DONE":  # Transition to SAMPLE_STATE
                    self.current_state.on_exit()
                    self.current_state = PrintState("PRINT_STATE", self.required_nodes, self.get_logger(), self, self.stage_pub)
                    self.current_state.on_enter()
                    self.get_logger().info("Transitioning to PRINT_STATE.")

                elif result == "PRINT_CYCLE_DONE":  # Transition to SAMPLE_STATE
                    self.current_state.on_exit()
                    self.current_state = ScanState("SCAN_STATE", self.required_nodes, self.get_logger(), self, self.stage_pub)
                    self.current_state.on_enter()
                    self.get_logger().info("Transitioning to SCAN_STATE.")
                    self.print_cycles += 1  # Increment print cycle count
                    self.publish_print_cycles()  # Publish updated count

                elif result == "samplestate":  # Transition to SAMPLE_STATE
                    self.current_state.on_exit()
                    self.current_state = State("SAMPLE_STATE")
                    self.current_state.on_enter()
                    self.get_logger().info("Transitioning to SAMPLE_STATE.")

                elif result == "reset":  # Restart the state machine
                    self.current_state.on_exit()
                    self.current_state = StartupState("STARTUP_STATE", self.required_nodes, self.get_logger(), self, self.stage_pub)
                    self.current_state.on_enter()
                    self.get_logger().info("Resetting to STARTUP_STATE.")
                else:
                    log_msg.data = result

            state_msg.data = self.current_state.name

        # Publish the messages
        self.status_pub.publish(status_msg)
        self.log_pub.publish(log_msg)
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"Published Status: {status_msg.data}")
        self.get_logger().info(f"Published Log: {log_msg.data}")

    def goto_state(self, target_state_name):
        """Forcefully transition to a specified state, resetting the stage."""
        state_mapping = {
            "STARTUP_STATE": StartupState,
            "SCAN_STATE": ScanState,
            "PRINT_STATE": PrintState,
            "SAMPLE_STATE": State, 
        }

        # Check if the target state exists in the mapping
        if target_state_name not in state_mapping:
            self.get_logger().error(f"Invalid state: {target_state_name}")
            return

        # Exit the current state if it exists
        if self.current_state is not None:
            self.current_state.on_exit()
            self.get_logger().info(f"Exiting current state: {self.current_state.name}")

        # Instantiate the new state
        new_state_class = state_mapping[target_state_name]
        if target_state_name in ["STARTUP_STATE", "SCAN_STATE", "PRINT_STATE"]:
            self.current_state = new_state_class(target_state_name, self.required_nodes, self.get_logger(), self, self.stage_pub)
        else:
            self.current_state = new_state_class(target_state_name)


        # Reset the stage and enter the new state
        self.current_state.stage = 1
        self.current_state.on_enter()
        self.get_logger().info(f"Transitioned to {target_state_name} with stage reset to 1")

        # Publish the initial stage
        self.publish_status()
        self.publish_none_stage()

    def listen_for_input(self):
        """Listen for user input in the terminal and handle commands."""
        def input_thread():
            while True:
                user_input = input("Press Enter to start when in NONE state, or type commands: ").strip()
                if user_input == "" and self.current_state is None:
                    self.start()
                elif user_input.startswith("goto "):
                    target_state_name = user_input.split(" ", 1)[1].upper()
                    self.goto_state(target_state_name)
                elif user_input == "pause":
                    self.pause()
                else:
                    self.get_logger().info(f"Unknown command: {user_input}")

        thread = threading.Thread(target=input_thread, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()
