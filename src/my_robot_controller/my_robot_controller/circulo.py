#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircle(Node):

    def __init__(self):

        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5,self.send_vel)
        self.get_logger().info("Iniciado el dibujeitor")

    def send_vel(self):
        msg = Twist()
        msg.linear.x = 4.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()