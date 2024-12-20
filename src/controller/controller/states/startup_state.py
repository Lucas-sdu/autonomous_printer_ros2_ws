import subprocess
import time 
from std_msgs.msg import String

class StartupState:
    def __init__(self, name, node_names, logger,node,stage_pub):
        self.name = name
        self.stage = 1
        self.counter = 0
        self.delay = 5.0
        self.required_nodes = node_names
        self.node = node
        self.checked_nodes = []
        self.logger = logger
        self.stage_pub = stage_pub  # Stage publisher

    def on_enter(self):
        self.counter = 0
        self.checked_nodes = []
        self.logger.info(f"Entering state: {self.name}. Checking for required nodes...")
        
    def execute(self, active_nodes):
        """
        Check if all required nodes are running and confirm connection.
        """
        self.counter += 1
        self.publish_stage() 
        # Stage 1: Start all required nodes
        if self.stage == 1:
            missing_nodes = []

            # Check for missing nodes
            for node in self.required_nodes:
                if node not in active_nodes:
                    if node not in self.checked_nodes:
                        self.logger.info(f"Starting node '{node}'...")
                        self.start_node(node)
                        self.checked_nodes.append(node)

                    missing_nodes.append(node)
            if not missing_nodes:
                self.logger.info("Stage 1 Complete: All nodes are running.")
                self.stage += 1  # Move to Stage 2
                self.publish_stage() 
            else:
                return "Waiting for all nodes to start..."
            
        # Stage 2: Communication test with Grasshopper and Printer
        elif self.stage == 2:
            if not hasattr(self, 'is_gh_connected'):
                self.is_gh_connected = False
                self.is_printer_connected = False
                self.g28_sent = False
                self.g28_acknowledged = False
                self.logger.info("Stage 2: Starting communication test...")

            # Update flags based on shared data
            grasshopper_msg = self.node.shared_data.get("grasshopper_input")
            printer_status = self.node.shared_data.get("printer_status")

            # Confirm Grasshopper sends G28
            if grasshopper_msg == "G28" and not self.g28_sent:
                self.is_gh_connected = True
                self.g28_sent = True  # Mark G28 as sent
                self.logger.info("Grasshopper connection confirmed. G28 sent.")

            # Confirm Printer OK, but only after G28 has been sent and acknowledged
            if self.g28_sent and not self.g28_acknowledged:
                if printer_status and "ok" in printer_status.lower():
                    self.g28_acknowledged = True  # G28 acknowledged by printer
                    self.logger.info("Printer acknowledged G28 command with OK.")

            # Confirm Printer OK after G28 acknowledgment
            if self.g28_acknowledged and printer_status and "ok" in printer_status.lower():
                self.is_printer_connected = True
                self.logger.info("Printer connection confirmed (OK after G28).")

            # Check if both conditions are satisfied
            if self.is_gh_connected and self.is_printer_connected:
                self.logger.info("Stage 2 Complete: Communication test successful.")
                self.stage += 1  # Move to Stage 3
                self.publish_stage()
                del self.is_gh_connected
                del self.is_printer_connected
                del self.g28_sent
                del self.g28_acknowledged
            else:
                return "Waiting for Grasshopper 'G28' and Printer 'OK'..."


        # Stage 3: wait for heating
        elif self.stage == 3:
            # Initialize flags if not already set
            if not hasattr(self, 'received_m109'):
                self.received_m109 = False
                self.received_printer_ok = False
                self.received_printer_lcd = False
                self.logger.info("Stage 3: Waiting for Grasshopper 'M109' command and Printer 'OK'...")

            # Check shared data for Grasshopper input
            grasshopper_msg = self.node.shared_data.get("grasshopper_input")
            printer_status = self.node.shared_data.get("printer_status")

            if not self.received_m109 and grasshopper_msg and "m109" in grasshopper_msg.lower():
                self.received_m109 = True
                self.logger.info("Grasshopper sent M109. Waiting for printer confirmation...")


            # Detect LCD change from Printer after M109
            if self.received_m109 and not self.received_printer_lcd:
                if printer_status and "lcd status" in printer_status.lower():
                    self.received_printer_lcd = True
                    self.logger.info("Printer acknowledged 'change' after M109.")
                        # Detect LCD change from Printer after M109
            if self.received_m109 and self.received_printer_lcd and not self.received_printer_ok:
                if printer_status and "ok" in printer_status.lower():
                    self.received_printer_ok = True
                    self.logger.info("Printer acknowledged 'ok' after M109.")

            # Check both conditions to move to Stage 4
            if self.received_m109 and self.received_printer_ok and self.received_printer_lcd:
                self.logger.info("Stage 3 Complete: M109 sent and Printer acknowledged OK.")
                self.stage += 1  # Move to Stage 4
                self.publish_stage() 
                del self.received_m109  # Clean up flags
                del self.received_printer_ok
            else:
                return "Waiting for Grasshopper 'M109' and Printer 'OK'..."

        # Final stage: Done
        if self.stage > 3:
            self.logger.info("Startup State Complete: All stages successful. Press Enter to proceed...")
            input()  # Wait for user input to proceed
            self.stage = 0  #Reset stage
            self.publish_stage()
            return "done"

        return f"Startup check counter: {self.counter}"

    def start_node(self, node_name):
        try:
            if node_name == "gcode_manager":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gcode_serial gcode_manager --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
                time.sleep(self.delay)
                active_nodes = self.node.get_node_names()
                self.logger.info(f"Updated Active Nodes: {active_nodes}")
            elif node_name == "ros_to_gh":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gh_ros ros_to_gh; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
                time.sleep(self.delay)
                active_nodes = self.node.get_node_names()
                self.logger.info(f"Updated Active Nodes: {active_nodes}")
            elif node_name == "gh_subscriber":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gh_ros gh_subscriber; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
                time.sleep(self.delay)
                active_nodes = self.node.get_node_names()
                self.logger.info(f"Updated Active Nodes: {active_nodes}")
            elif node_name == "rosbridge_websocket":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
                time.sleep(self.delay)
                active_nodes = self.node.get_node_names()
                self.logger.info(f"Updated Active Nodes: {active_nodes}")
            elif node_name == "image_manager":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run image_manager image_manager; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
                time.sleep(self.delay)
                active_nodes = self.node.get_node_names()
                self.logger.info(f"Updated Active Nodes: {active_nodes}")
        except Exception as e:
            self.logger.error(f"Failed to start node '{node_name}': {str(e)}")

    def on_exit(self):
        self.logger.info(f"Exiting state: {self.name}")
    
    def publish_stage(self):
        """Publish the current state and stage."""
        stage_msg = String()
        stage_msg.data = f"{self.stage}"
        self.stage_pub.publish(stage_msg)
