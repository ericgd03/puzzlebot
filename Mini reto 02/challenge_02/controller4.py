#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

from math import sqrt
from math import atan2

class twister(Node):

    def __init__(self):
        super().__init__('twist')

        #Valores que determinan si se llego al obj
        self.disDone = False
        self.angDone = False

        #Valores que recibe del publisher path
        self.objX = 0.0
        self.objY = 0.0
        
        #Valores del Pose2D para el movimiento
        self.pose_msg = Pose2D()
        self.pose_msg.x = 0.0
        self.pose_msg.y = 0.0
        self.pose_msg.theta = 0.0

        #Valores del twist
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0

        #Posicion actual del rbot y distancia con nueva coordenada
        self.posX = 0.0
        self.posY = 0.0

        self.kp_linear = 0.1
        self.kp_angular = 0.1

        self.posAng = 0.0 
        self.posDis = 0.0

        self.dis = 0.0  
        self.aang = 0.0

        #Valores del encoder
        self.encL = 0.0
        self.encR = 0.0
        
        #Subscripiones de path, encoder izquierdo y derecho
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.odom_sub = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile)
        self.point_sub = self.create_subscription(Pose2D, 'point', self.point_callback, qos_profile)

        #Crea los publisher a la velocidad linear y angular
        self.puzzlebot_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_publisher = self.create_publisher(Twist, 'signal', 10)
        self.get_logger().info('Nodes initialized!')       

        #Llama a publicar cada 0.1 segundos
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
         
        #Calcula la distancia entre puntos y su error
        self.disError = sqrt( pow(self.objX - self.pose_msg.x,2) + pow(self.objY - self.pose_msg.y,2) )
        print('Posiciones')
        print(self.pose_msg.x)
        print(self.pose_msg.y)

        print('Error de Distancia = ')
        print(self.disError)

        #Calculo el angulo y su error
        self.angError = atan2(self.objY - self.pose_msg.y, self.objX - self.pose_msg.x)

        print('Error de Angulo')
        print(self.angError)
        #Si el error de distancia es menor a 0.01 el robot dejara de avanzar                
        if(self.disError < 0.2) and (self.disError > -0.2):
                self.twist_msg.linear.x = 0.0
                self.disDone = True
        else:
                self.twist_msg.linear.x = self.kp_linear * self.disError
        
        #Si el error de distancia es menor a 0.01 el robot dejara de girar
        if(self.angError < 0.5) and (self.angError > -0.5):
                self.twist_msg.angular.z = 0.0
                self.angDone = True
        else:
                self.twist_msg.angular.z = self.kp_angular * self.angError
        
        if(self.disDone) and (self.angDone):
              self.done_publisher.publish(self.twist_msg)
              self.twist_msg.angular.z = 0.0
              self.twist_msg.linear.x = 0.0

        #Publicar a robot
        print('Velocidad angular y linear')
        print(self.twist_msg.linear.x)
        print(self.twist_msg.angular.z)
        self.puzzlebot_publisher.publish(self.twist_msg)

    #Recibe datos del odometria
    def odom_callback(self, msg):
        self.pose_msg.x = msg.x
        self.pose_msg.y = msg.y
        self.pose_msg.theta = msg.theta

        print('Valores de odometria')
        print(msg.x)
        print(msg.y)
        print(msg.theta)
    
    #Recibe la coordenada objetivo
    def point_callback(self, msg):
        self.objX = msg.x
        self.objY = msg.y

        print('Valores de coordenada')
        print(self.objX)
        print(self.objY)

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