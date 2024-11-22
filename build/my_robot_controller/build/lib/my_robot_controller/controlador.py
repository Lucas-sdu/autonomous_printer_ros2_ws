#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class ControladorTortuga(Node):

    def __init__(self):
        super().__init__("controlador")
        self.previous_x = 0
        self.cmd_vel_publish = self.create_publisher( Twist,"/turtle1/cmd_vel",10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_call,10)
        self.get_logger().info("Ponele weeeno chuchesumadre")

    def pose_call(self, pose: Pose):
        cmd = Twist()
        
        if pose.x> 9.0 or pose.x< 2.0 or pose.y> 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publish.publish(cmd)

        if pose.x > 5.5 and self.previous_x <= 5.5:
            self.get_logger().info("Tamos Red...")
            self.call_pen(255,0,0,3,0)
            self.previous_x = pose.x
        elif pose.x <= 5.5 and self.previous_x > 5.5:
            self.get_logger().info("Tamos Green...")
            self.call_pen(0,220,10,3,0)
            self.previous_x = pose.x

    def call_pen(self,r,g,b,width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.5):
            self.get_logger().warn("Esperando como los gile...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.call_set_pen))

    def call_set_pen(self,future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Cagaaamos! %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = ControladorTortuga()
    rclpy.spin(node)
    rclpy.shutdown()