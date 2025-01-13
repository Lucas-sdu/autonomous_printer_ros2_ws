import subprocess
import time 
from std_msgs.msg import String
class ScanState:
    def __init__(self, name, node_names, logger,node,stage_pub):

        self.name = name
        self.stage = 1  # Start with Stage 1
        self.required_nodes = node_names
        self.node = node
        self.checked_nodes = []
        self.logger = logger
        self.stage_pub = stage_pub  # Stage publisher
        self.target_position = {"X": 0.0, "Y": 0.0, "Z": 200.0}  # Target position
        self.start_time = None
        # Create a publisher for the /grasshopper_input topic
        self.grasshopper_input_pub = self.node.create_publisher(String, "/grasshopper_input", 10)
    def on_enter(self):
        print(f"Entering state: {self.name}")
        self.start_time = time.time()  # Record start time
        self.logger.info("Stage 1: Preparing to check position.")
        self.stage = 1

        # Reset last_actual_position to None
        self.node.shared_data["last_actual_position"] = None
        grasshopper_input = String()
        grasshopper_input.data = "M114"
        self.grasshopper_input_pub.publish(grasshopper_input)
        self.publish_stage() 
    def execute(self):
        self.publish_stage() 
        if self.stage == 1:
            elapsed_time = time.time() - self.start_time if self.start_time else 0

            # Resend M114 if 5 seconds have passed
            if elapsed_time >= 5 or not self.start_time:
                self.logger.info("Stage 1: Sending M114 command to request position.")
                grasshopper_input = String()
                grasshopper_input.data = "M114"
                self.grasshopper_input_pub.publish(grasshopper_input)
                self.start_time = time.time()  # Reset timer after sending M114
                return "waiting"

            # Check for the position response
            last_actual_position = self.node.shared_data.get("last_actual_position")
            if last_actual_position:
                self.logger.info(f"Stage 1: Received position {last_actual_position}. Comparing with target...")
                is_within_tolerance = all(
                    abs(last_actual_position[axis] - self.target_position[axis]) <= 2.0
                    for axis in self.target_position
                )

                if is_within_tolerance:
                    self.logger.info("Stage 1: Position is within tolerance. Moving to Stage 2.")
                    self.stage += 1  # Move to Stage 2
                    return "waiting"
                else:
                    self.logger.info("Stage 1: Position out of tolerance. Continuing to poll.")
                    return "waiting"

            self.logger.info(f"Stage 1: Waiting for position response... {elapsed_time:.2f} seconds elapsed.")
            return "waiting"
        
        # Stage 2: Wait for 'IMG_DONE' message from Grasshopper
        elif self.stage == 2:
            self.publish_stage()

            # Check for 'IMG_DONE' in Grasshopper input
            grasshopper_input = self.node.shared_data.get("grasshopper_input")
            
            if grasshopper_input and "IMG_DONE" in grasshopper_input:
                self.node.get_logger().info("Stage 2 Complete: 'IMG_DONE' message received from Grasshopper.")
                self.node.shared_data["grasshopper_input"] = None  
                self.stage += 1  # Move to the next stage
                
                return "waiting"

            # Default: keep waiting
            self.node.get_logger().info("Stage 2: Waiting for 'IMG_DONE' from Grasshopper...")
            return "waiting"

        # Stage 3: Wait for 'DONE' message from Grasshopper and user to press Enter
        elif self.stage == 3:
            self.publish_stage()

            # Check for Grasshopper input
            grasshopper_input = self.node.shared_data.get("grasshopper_input")
            
            if grasshopper_input and "DONE" in grasshopper_input:
                self.node.get_logger().info("Stage 3: 'DONE' message received from Grasshopper. PRESS ENTER")
                
                # Now wait for Enter from the terminal
                try:
                    user_input = input("Presiona Enter para confirmar y continuar: ").strip()
                    if user_input == "":
                        self.node.get_logger().info("Stage 3 Complete: Both Grasshopper 'DONE' and Enter confirmed.")
                        return "SCAN_DONE"  # Signal to move to the next state
                except EOFError:
                    self.node.get_logger().warn("Stage 3: No input available in terminal for Enter confirmation.")
            
            # Default: keep waiting
            self.node.get_logger().info("Stage 3: Waiting for 'DONE' from Grasshopper and Enter in terminal...")
            return "waiting"

    def on_exit(self):
        print(f"Exiting state: {self.name}")
        self.stage = 0  # Reset the stage to 0
        self.node.shared_data["last_actual_position"] = None  
        self.publish_stage() 
    def publish_stage(self):
        """Publish the current state and stage."""
        stage_msg = String()
        stage_msg.data = f"{self.stage}"
        self.stage_pub.publish(stage_msg)
""" Example for sending commands to go to different state
    def execute(self):
        user_input = input("Waiting for input (Enter = samplestate, r = reset): ").strip()
        if user_input == "":
            return "samplestate"  # Enter key sends "next"
        elif user_input.lower() == "r":
            return "reset"  # "r" key sends "reset"
        else:
            print("Invalid input. Try again.")
            return "waiting"  # Continue waiting for valid input
 """
