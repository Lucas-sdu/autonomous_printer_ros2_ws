#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose


class elsubscriptor(Node):

    def __init__(self):

        super().__init__("subscribidor")
        self.subscriptor =self.create_subscription(Pose, "/turtle1/pose",self.callback_pos,10)
    def callback_pos(self,msg: Pose):
        self.get_logger().info("("+str(msg.x)+","+str(msg.y)+")")

def main(args=None):
    rclpy.init(args=args)
    node = elsubscriptor()
    rclpy.spin(node)
    rclpy.shutdown()