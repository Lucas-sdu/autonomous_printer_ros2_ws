#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .states.startup_state import StartupState
from .states.placeholder_state import State

# Controller Node
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Initialized")

        # Required nodes for startup
        self.required_nodes = ["gcode_manager"]

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
        self.state_pub = self.create_publisher(
            String, '/controller_state', 10)  # New publisher for current state
        
        # Timer to continuously publish status and state
        self.timer = self.create_timer(1.0, self.publish_status)

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

    def start(self):
        if self.current_state is None:
            # Start in StartupState
            self.current_state = StartupState("STARTUP_STATE", self.required_nodes, self.get_logger())
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
        self.get_logger().info("State machine reset")

    def publish_status(self):
        status_msg = String()
        log_msg = String()
        state_msg = String()  # New state message
        active_nodes = self.get_node_names()  # Retrieve active nodes in the graph

        if self.current_state is None:
            status_msg.data = "State: NONE | Status: IDLE"
            log_msg.data = "LOG: No active state."
            state_msg.data = "NONE"  # Publish NONE as the current state
        elif self.is_paused:
            status_msg.data = f"State: {self.current_state.name} | Status: PAUSED"
            log_msg.data = f"LOG: {self.current_state.name} paused."
            state_msg.data = self.current_state.name  # Publish the current state name
        else:
            status_msg.data = f"State: {self.current_state.name} | Status: RUNNING"
            state_msg.data = self.current_state.name  # Publish the current state name
            if isinstance(self.current_state, StartupState):
                result = self.current_state.execute(active_nodes)
            else:
                result = self.current_state.execute()
                
            if result == "done":
                # Transition from StartupState to Placeholder State
                self.current_state.on_exit()
                self.current_state = State("SAMPLE_STATE")
                self.current_state.on_enter()
                log_msg.data = "LOG: Transitioned to SAMPLE_STATE."
            else:
                log_msg.data = result

        self.status_pub.publish(status_msg)
        self.log_pub.publish(log_msg)
        self.state_pub.publish(state_msg)  # Publish the current state
        self.get_logger().info(f"Published Status: {status_msg.data}")
        self.get_logger().info(f"Published Log: {log_msg.data}")
        self.get_logger().info(f"Published State: {state_msg.data}")


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
