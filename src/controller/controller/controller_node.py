#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .states.startup_state import StartupState
from .states.scan_state import ScanState
from .states.placeholder_state import State

# Controller Node
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Initialized")

        # Shared message store
        self.shared_data = {"grasshopper_input": None, "printer_status": None} 
        #Check gh connection
        self.connected_to_grasshopper = False
        
        # Required nodes for startup
        self.required_nodes = ["gcode_manager", "ros_to_gh","gh_subscriber","rosbridge_websocket","image_manager"]

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

        
        
        #Stage Publisher
        self.stage_pub = self.create_publisher(String, '/controller_stage', 10)  
        self.publish_none_stage()  # Publish NONE stage only at startup

        # Timer to continuously publish status and state
        self.timer = self.create_timer(1.0, self.publish_status)

        # Add the subscriber to listen for Grasshopper messages
        self.grasshopper_sub = self.create_subscription(
            String,
            '/grasshopper_input',
            self.grasshopper_callback,
            10
        )
        self.printer_status_sub = self.create_subscription(
            String, '/printer_status', self.printer_status_callback, 10)
        
    def grasshopper_callback(self, msg):
        """Store messages from Grasshopper in shared data"""
        self.shared_data["grasshopper_input"] = msg.data
        self.get_logger().info(f"Received message on '/grasshopper_input': {msg.data}")
    def printer_status_callback(self, msg):
        """Store Printer OK response in shared data."""
        self.shared_data["printer_status"] = msg.data
        self.get_logger().info(f"Received printer status: {msg.data}")
    def command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")

        if command == 'start':
            self.start()
        elif command == 'pause':
            self.pause()
        elif command == 'reset':
            self.reset()
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

    def reset(self):
        if self.current_state is not None:
            self.current_state.on_exit()
        self.current_state = None
        self.is_paused = False
        self.publish_none_stage()
        self.get_logger().info("State machine reset")

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
                if isinstance(self.current_state, StartupState):
                    result = self.current_state.execute(active_nodes)
                else:
                    result = self.current_state.execute()

                if result == "done":
                    self.current_state.on_exit()
                    self.current_state = ScanState("SCAN_STATE")
                    self.current_state.on_enter()
                    self.get_logger().info("Transitioning to SCAN_STATE.")

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

if __name__ == '__main__':
    main()
