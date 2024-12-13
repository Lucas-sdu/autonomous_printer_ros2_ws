#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Startup State Class
class StartupState:
    def __init__(self, name):
        self.name = name
        self.counter = 0  # Simple condition counter
    
    def on_enter(self):
        print(f"Entering state: {self.name}")
        self.counter = 0

    def execute(self):
        self.counter += 1
        print(f"Executing state: {self.name} | Counter: {self.counter}")
        # Placeholder condition: Transition after counter reaches 5
        if self.counter >= 5:
            return "done"
        return f"Startup counter: {self.counter}"

    def on_exit(self):
        print(f"Exiting state: {self.name}")

# Placeholder State Class
class State:
    def __init__(self, name):
        self.name = name
        self.counter = 0  # Add counter
        
    def on_enter(self):
        print(f"Entering state: {self.name}")

    def execute(self):
        self.counter += 1
        print(f"Executing state: {self.name} | Counter: {self.counter}")
        # Placeholder logic for the state execution
        return f"Counter value: {self.counter}"

    def on_exit(self):
        print(f"Exiting state: {self.name}")

# Controller Node
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller Node Initialized")

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

        # Timer to continuously publish status
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
            self.current_state = StartupState("STARTUP_STATE")
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
        if self.current_state is None:
            status_msg.data = "State: NONE | Status: IDLE"
            log_msg.data = "LOG: No active state."
        elif self.is_paused:
            status_msg.data = f"State: {self.current_state.name} | Status: PAUSED"
            log_msg.data = f"LOG: {self.current_state.name} paused."
        else:
            status_msg.data = f"State: {self.current_state.name} | Status: RUNNING"
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
