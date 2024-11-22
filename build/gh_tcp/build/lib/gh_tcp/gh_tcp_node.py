#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GHTcpNode(Node):
    def __init__(self):
        super().__init__('gh_tcp_node')
        self.publisher_ = self.create_publisher(String, 'grasshopper_data', 10)

        # Set up the TCP server
        self.tcp_ip = '0.0.0.0'  # Listen on all available network interfaces
        self.tcp_port = 4210     # Port to match Grasshopper's TCP Client
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.tcp_ip, self.tcp_port))
        self.server_socket.listen(1)  # Accept only one connection at a time

        self.get_logger().info(f"TCP Server listening on {self.tcp_ip}:{self.tcp_port}")

        # Accept a client connection
        self.client_socket, self.client_address = self.server_socket.accept()
        self.get_logger().info(f"Connection established with {self.client_address}")

        # Timer to continuously listen for incoming data
        self.timer = self.create_timer(0.1, self.listen_for_data)

    def listen_for_data(self):
        try:
            data = self.client_socket.recv(1024).decode('utf-8')  # Buffer size: 1024 bytes
            if data:
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
                self.get_logger().info(f"Received from Grasshopper: {data}")
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")
            self.cleanup()

    def cleanup(self):
        self.get_logger().info("Closing connection")
        self.client_socket.close()
        self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = GHTcpNode()
    rclpy.spin(node)
    node.cleanup()  # Ensure cleanup is called on shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
