class State:
    def __init__(self, name):
        self.name = name
        self.counter = 0
        
    def on_enter(self):
        print(f"Entering state: {self.name}")

    def execute(self):
        self.counter += 1
        print(f"Executing state: {self.name} | Counter: {self.counter}")
        return f"Counter value: {self.counter}"

    def on_exit(self):
        print(f"Exiting state: {self.name}")
