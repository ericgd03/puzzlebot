#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

class My_Publisher(Node):
    def __init__(self):
        super().__init__('Generator_node')
        #Variables de las figuras
        self.puntoAct = 0

        self.triangX = [0.0, 0.5]
        self.triangY = [0.5, 0.5]
        self.triangDone = False
        
        #Crea el suscriber y el publisher
        self.publisher = self.create_publisher(Pose2D, 'point', 10)
        self.sub = self.create_subscription(Twist, 'signal', self.listener_callback, 10)
        self.get_logger().info('Pointer generator node successfully initialized!')
        self.msg = Pose2D()
    
    def listener_callback(self, msg):
        if (not self.triangDone):
            self.get_logger().info('Realizando Triangulo')
            self.msg.x = self.triangX[self.puntoAct]
            self.msg.y = self.triangY[self.puntoAct]
            self.puntoAct = self.puntoAct + 1
            if(self.puntoAct > 1):
                self.triangDone = True
                self.puntoAct = 0
        else:
            self.msg.x = 0.0
            self.msg.y = 0.0
        
        print('Objetivo')
        print(self.msg.x)
        print(self.msg.y)
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()