#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
i = 0
contador = 0
giro = False
avanza = False

class twister(Node):

    def __init__(self):
        super().__init__('twist')
        global datax
        global datay
        global msg
        self.sub = self.create_subscription(String, 'pose', self.listener_callback, 10)
          
        self.datax = [0.0, -1.0, -1.0, -1.0]
        self.datay = [0.0, -1.0, -1.0, -1.0]
        self.msg = Twist()
        self.msg.linear.x = self.datax[0]
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = self.datay[0]
                
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info('Twister node initialized!')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global msg
        global contador
        global giro
        global avanza
        
        if(giro):
             self.msg.linear.x = 0.0
             self.msg.angular.z = self.datay[0]
             contador = contador + 1
             if contador >= 1:
                  giro = False
                  contador = 0
             print(self.msg.linear.x)
             print(self.msg.angular.z)
        elif(avanza):
             self.msg.linear.x = self.datax[0]
             self.msg.angular.z = 0.0    
             contador = contador + 10
             if contador > 45:
                  avanza = False
                  contador = 0               
        else:
             self.msg.linear.x = 0.0
             self.msg.angular.z = 0.0   
                       
        self.publisher_.publish(self.msg)

    def listener_callback(self, msg):
        global i
        global datax
        global datay
        global giro
        global contador
        global avanza
        
        self.get_logger().info('Data: {}'.format(msg.data))
        mensaje = msg.data
        datos = mensaje.split(",")
        self.datax[0] = float(datos[0])
        self.datay[0] = float(datos[1])
        
        giro = True
        avanza = True
        contador = 0
        
        i = i + 1
        print(i)
        print(self.datax)
        print(self.datay)


def main(args=None):
    rclpy.init(args=args)
    
    twist = twister()
    rclpy.spin(twist)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
	main()
