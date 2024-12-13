import subprocess

class StartupState:
    def __init__(self, name, node_names, logger):
        self.name = name
        self.counter = 0
        self.required_nodes = ["gcode_manager", "ros_to_gh","gh_subscriber"]

        self.checked_nodes = []
        self.logger = logger

    def on_enter(self):
        self.counter = 0
        self.checked_nodes = []
        self.logger.info(f"Entering state: {self.name}. Checking for required nodes...")

    def execute(self, active_nodes):
        """
        Check if all required nodes are running before transitioning to the next state.
        """
        self.counter += 1
        missing_nodes = []

        # Check for missing nodes
        for node in self.required_nodes:
            if node not in active_nodes:
                if node not in self.checked_nodes:
                    self.logger.warn(f"Node '{node}' not found. Attempting to start it...")
                    self.start_node(node)
                    self.checked_nodes.append(node)  # Avoid restarting the same node
                missing_nodes.append(node)

        # If all required nodes are running, signal 'done'
        if not missing_nodes:
            self.logger.info("All required nodes are running. Transitioning to the next state.")
            return "done"

        # Log missing nodes
        self.logger.info(f"Waiting for nodes: {', '.join(missing_nodes)}")
        return f"Startup check counter: {self.counter}"

    def start_node(self, node_name):
        try:
            if node_name == "gcode_manager":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gcode_serial gcode_manager --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
            elif node_name == "ros_to_gh":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gh_ros ros_to_gh; exec bash"
                ])
            elif node_name == "gh_subscriber":
                subprocess.Popen([
                    "gnome-terminal", "--", "bash", "-c",
                    "ros2 run gh_ros gh_subscriber; exec bash"
                ])
                self.logger.info(f"Successfully launched node: {node_name} in a new terminal.")
        
        except Exception as e:
            self.logger.error(f"Failed to start node '{node_name}': {str(e)}")

    def on_exit(self):
        self.logger.info(f"Exiting state: {self.name}")
