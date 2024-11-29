import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('ros_publisher')
        self.publisher_ = self.create_publisher(String, '/ros_to_gh', 10)
        self.counter = 0
        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('ROS Publisher Node has been started (placeholder)')

    def timer_callback(self):
        self.counter += 1
        msg = String()
        msg.data = f'Hola grasshopper este es el mensaje numero: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    ros_publisher = ROSPublisher()
    try:
        rclpy.spin(ros_publisher)
    except KeyboardInterrupt:
        pass
    ros_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
