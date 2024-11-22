#!/usr/bin/env python3
import rclpy #libreria de ros
from rclpy.node import Node #trae los parametros de la clase node de la libreria?

class MyNode(Node):
 
    def __init__(self): #Constructor de la clase
        super().__init__("first_node")#el nombre del real nodo que aparece en la gráfica rqt_graph
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("wena compare!"+  str(self.counter_))
        self.counter_ += 1

def main (args=None):
    rclpy.init(args=args)#Inicia comunicacion ros2
    node = MyNode() #creamos el nodocon la clase de arriba
    rclpy.spin(node)#esto mantiene al nodo corriendo constantemente
    #
    rclpy.shutdown()#cierra comunicacion ros2

if __name__ == '__main__':
    main()