#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

class Nodo(Node):
    def __init__(self):
        super().__init__("segundo_nodo")
        self.create_timer(1.0,self.timer_callback)
    def timer_callback(self):
        self.get_logger().info("Hola")
def main (args=None):
    rclpy.init(args=args)

    node = Nodo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()