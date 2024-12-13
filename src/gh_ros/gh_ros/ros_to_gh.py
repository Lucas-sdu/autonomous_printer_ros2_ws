#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('ros_publisher')
        
        # Declare and initialize the parameter
        self.declare_parameter('set_time', 10.0)
        self.timer_period = self.get_parameter('set_time').value  # Extract the initial value as a float
        self.timer = None

        # Set up a parameter callback to monitor changes
        self.add_on_set_parameters_callback(self.set_time_callback)

        # Publisher to Grasshopper
        self.publisher_ = self.create_publisher(String, '/ros_to_gh', 10)
        
        # Subscriber to Controller State Updates
        self.subscription = self.create_subscription(
            String,
            '/controller_state',
            self.state_callback,
            10
        )
        
        self.counter = 0
        self.state = None  # Default state is None

        # Create the initial timer
        self.create_timer_with_period(self.timer_period)
        self.get_logger().info('ROS Publisher Node has been started')

    def set_time_callback(self, params):
        for param in params:
            if param.name == 'set_time' and param.type_ == param.Type.DOUBLE:
                new_timer_period = param.value
                if new_timer_period > 0.0 and new_timer_period != self.timer_period:
                    self.timer_period = new_timer_period
                    self.get_logger().info(f"Timer period updated to: {self.timer_period} seconds")
                    self.create_timer_with_period(self.timer_period)
                elif new_timer_period <= 0.0:
                    self.get_logger().warn("Invalid timer period value. Keeping the previous value.")
        return SetParametersResult(successful=True)

    def create_timer_with_period(self, period):
        if self.timer is not None:
            self.timer.destroy()
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f"Created a timer with period: {period}s")

    def state_callback(self, msg):
        """ Callback for receiving state updates from the controller node """
        self.state = msg.data
    #    self.get_logger().info(f"Updated state: {self.state}")

    def timer_callback(self):
        self.counter += 1
        msg = String()
        # Default to "none" if state is not yet set
        state_str = self.state if self.state else "none"
        msg.data = f'Hola_grasshopper State:{state_str} Counter:{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    ros_publisher = ROSPublisher()
    try:
        rclpy.spin(ros_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ros_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
