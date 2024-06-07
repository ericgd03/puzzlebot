#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

from math import sqrt
from math import atan2
from math import pi

class twister(Node):

    def __init__(self):
        super().__init__('twist')

        #Valores que determinan si se llego al objetivo
        self.recibido = False
        self.alto = False

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

        self.kp_linear = 0.2
        self.kp_angular = 0.1

        self.posAng = 0.0 

        self.ang = 0.0

        #Valores del encoder
        self.encL = 0.0
        self.encR = 0.0
        
        #Subscripiones de path, encoder izquierdo y derecho
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.odom_sub = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile)
        self.angular_sub = self.create_subscription(String, 'angular', self.angular_callback, 10)
        self.semaforo_sub = self.create_subscription(String, 'semaforo', self.semaforo_callback, 10)

        #Crea los publisher a la velocidad linear y angular
        self.puzzlebot_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_publisher = self.create_publisher(Twist, 'signal', 10)
        self.get_logger().info('Nodos initialized!')       

        #Llama a publicar cada 0.1 segundos
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        #Imprime la pos del puzzlebot
        log_msg = 'Pos X: {:.3f} Pos Y: {:.3f} Theta {:.3f}'.format(self.pose_msg.x, self.pose_msg.y,self.pose_msg.theta)
        self.get_logger().info(log_msg)

        #Calcula la distancia entre puntos y su error, igual con el angulo
        self.angError =  self.ang * (-1)

        log_msg = 'Dire Angular: {:.3f}'.format(self.angError) 
        self.get_logger().info(log_msg)

        #Establecer valor
        self.twist_msg.linear.x = self.kp_linear * 0.5
        self.twist_msg.angular.z = self.kp_angular * self.angError / 40

        #Publicar a robot
        log_msg = 'Vel ang: {:.3f} Vel lin: {:.3f}'.format(self.twist_msg.linear.x, self.twist_msg.angular.z)
        self.get_logger().info(log_msg)
        self.puzzlebot_publisher.publish(self.twist_msg)

    #Recibe datos del odometria
    def odom_callback(self, msg):
        self.pose_msg.x = msg.x
        self.pose_msg.y = msg.y
        self.pose_msg.theta = msg.theta

        #log_msg = 'Odo x: {:.3f} y: {:.3f} theta: {:.3f}'.format(msg.x, msg.y, msg.theta)
        #self.get_logger().info(log_msg)
    
    #Recibe el angulo de la camara
    def angular_callback(self, msg):
        self.get_logger().info(msg.data)
        mensaje = msg.data
        self.ang = float(mensaje)

    #Recibe el color del semaforo
    def semaforo_callback(self, msg):
        self.get_logger().info(msg.data)
        mensaje = msg.data

        if (mensaje == 'Green'):
            self.kp_angular = 0.2
            self.kp_linear = 0.2

        if (mensaje == 'Yellow'):
            self.kp_angular = 0.1
            self.kp_linear = 0.1

        if (mensaje == 'Red'):
            self.kp_angular = 0.0
            self.kp_linear = 0.0


            

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