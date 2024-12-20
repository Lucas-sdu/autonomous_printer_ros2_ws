class ScanState:
    def __init__(self, name):
        self.name = name

    def on_enter(self):
        print(f"Entering state: {self.name}")
        print("Press 'Enter' to proceed to SAMPLE_STATE or 'r' to reset to STARTUP_STATE.")

    def execute(self):
        user_input = input("Waiting for input (Enter = samplestate, r = reset): ").strip()
        if user_input == "":
            return "samplestate"  # Enter key sends "next"
        elif user_input.lower() == "r":
            return "reset"  # "r" key sends "reset"
        else:
            print("Invalid input. Try again.")
            return "waiting"  # Continue waiting for valid input

    def on_exit(self):
        print(f"Exiting state: {self.name}")
