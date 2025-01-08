import time
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class PrintState:
    def __init__(self, name, node_names, logger, node, stage_pub):
        self.name = name
        self.stage = 1  # Start with Stage 1
        self.required_nodes = node_names
        self.node = node
        self.logger = logger
        self.stage_pub = stage_pub  # Publisher for stage updates
        self.start_time = None
        self.position_confirmed_time = None
        self.m114_received = False
        self.target_position = {"X": 0.0, "Y": 0.0, "Z": 190.0}  # Target position

        # Create a publisher for the /grasshopper_input topic
        self.grasshopper_input_pub = self.node.create_publisher(String, "/grasshopper_input", 10)

    def on_enter(self):
        self.logger.info(f"Entering state: {self.name}")
        self.logger.info("Stage 1: Waiting for Grasshopper to send 'PRINT_DONE'.")
        self.stage = 1  # Reset the stage to 1
    def execute(self):
        self.publish_stage()

        if self.stage == 1:
            grasshopper_input = self.node.shared_data.get("grasshopper_input")
            if grasshopper_input == "PRINT_DONE":
                self.logger.info("Stage 1 Complete: Received 'PRINT_DONE' from Grasshopper.")
                self.start_time = None
                self.position_confirmed_time = None
                self.m114_received = False  # Reset M114 flag
                self.node.shared_data["printer_status"] = None
                self.node.shared_data["last_actual_position"] = None  # Clear last actual position
                self.stage += 1  # Move to Stage 2
                return "waiting"

            self.logger.info("Stage 1: Waiting for 'PRINT_DONE' from Grasshopper...")
            return "waiting"

        elif self.stage == 2:
            last_actual_position = self.node.shared_data.get("last_actual_position")

            if not self.m114_received:
                if last_actual_position:  # Check if M114 response has been parsed
                    self.logger.info(f"Stage 2: Received position {last_actual_position}. Comparing with target...")
                    is_within_tolerance = all(
                        abs(last_actual_position[axis] - self.target_position[axis]) <= 2.0 for axis in self.target_position
                    )

                    if is_within_tolerance:
                        self.logger.info("Stage 2: Position is within tolerance. Stage complete.")
                        return "PRINT_CYCLE_DONE"  # Signal to move to the next state

                    else:
                        self.logger.info("Stage 2: Position out of tolerance. Waiting to resend M114...")
                        self.start_time = time.time()
                        self.m114_received = True  # Mark as processed
                        return "waiting"
                else:
                    self.logger.info("Stage 2: Waiting for M114 position response...")
                return "waiting"

            if self.m114_received:
                elapsed_time = time.time() - self.start_time if self.start_time else 0
                if elapsed_time >= 5:  # Wait 5 seconds before resending M114
                    self.logger.info("Stage 2: Resending M114 command to Grasshopper input topic.")
                    grasshopper_input = String()
                    grasshopper_input.data = "M114"
                    self.grasshopper_input_pub.publish(grasshopper_input)
                    self.m114_received = False  # Reset flag to wait for new response
                    return "waiting"
                else:
                    self.logger.info(f"Stage 2: Waiting before resending M114... {elapsed_time:.2f} seconds elapsed.")

            return "waiting"

    def on_exit(self):
        self.logger.info(f"Exiting state: {self.name}")
        self.stage = 0  # Reset the stage to 0
        self.publish_stage() 
    def publish_stage(self):
        """Publish the current stage."""
        stage_msg = String()
        stage_msg.data = f"{self.stage}"
        self.stage_pub.publish(stage_msg)
