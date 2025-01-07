import time
from std_msgs.msg import String

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

    def on_enter(self):
        self.logger.info(f"Entering state: {self.name}")
        self.logger.info("Stage 1: Waiting for Grasshopper to send 'print_sent'.")

    def execute(self):
        self.publish_stage()

        if self.stage == 1:
            grasshopper_input = self.node.shared_data.get("grasshopper_input")
            if grasshopper_input == "print_sent":
                self.logger.info("Stage 1 Complete: Received 'print_sent' from Grasshopper.")
                self.stage += 1  # Move to Stage 2
                self.start_time = None
                self.position_confirmed_time = None
                return "waiting"

            self.logger.info("Stage 1: Waiting for 'print_sent' from Grasshopper...")
            return "waiting"
        
        elif self.stage == 2:
            printer_status = self.node.shared_data.get("printer_status")

            if printer_status and "ok" in printer_status.lower():
                if self.position_confirmed_time is None:
                    self.position_confirmed_time = time.time()
                    self.logger.info("Stage 2: Printer OK received. Starting stability timer.")

                elapsed_time = time.time() - self.position_confirmed_time
                if elapsed_time >= 10:  # Ensure stability
                    self.logger.info("Stage 2 Complete: Printer confirmed at safe position.")
                    return "PRINT_CYCLE_DONE"  # Signal to move to the next state
                else:
                    self.logger.info(f"Stage 2: Stability timer: {elapsed_time:.2f} seconds.")
            else:
                self.logger.info("Stage 2: Waiting for printer OK confirmation...")

            return "waiting"


    def on_exit(self):
        self.logger.info(f"Exiting state: {self.name}")

    def publish_stage(self):
        """Publish the current stage."""
        stage_msg = String()
        stage_msg.data = f"{self.stage}"
        self.stage_pub.publish(stage_msg)
