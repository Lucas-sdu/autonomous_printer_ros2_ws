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

    def on_enter(self):
        print(f"Entering state: {self.name}")
        print("Stage 1: Waiting for Grasshopper to move to scan position...")
        self.start_time = time.time()  # Record start time

    def execute(self):
        self.publish_stage() 
        if self.stage == 1:
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 8:
                self.logger.info("Stage 1 Complete: All nodes are running.")
                
                self.stage += 1  # Move to Stage 2
            return "waiting"
        
        # Stage 2: Check for 'Received image' from image_manager
        elif self.stage == 2:
            self.publish_stage()
            image_received = self.node.shared_data.get("image_input")
            
            if image_received:  # Check if an image has been marked as received
                self.node.get_logger().info("Stage 2 Complete: Image received.")
                self.stage += 1  # Move to the next stage
                self.node.shared_data["image_input"] = False  # Reset the flag
                return "waiting"
            
            self.node.get_logger().info("Stage 2: Waiting for image input...")
            return "waiteanding"

        # Stage 3: Wait for 'DONE' message from Grasshopper
        elif self.stage == 3:
            self.publish_stage() 
            grasshopper_input = self.node.shared_data.get("grasshopper_input")
            if grasshopper_input and "DONE" in grasshopper_input:
                self.node.get_logger().info("Stage 3 Complete: 'DONE' message received from Grasshopper.")
                return "SCAN_DONE"  # Signal to move to the next state
            return "waiting"

    def on_exit(self):
        print(f"Exiting state: {self.name}")

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
