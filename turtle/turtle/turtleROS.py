#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from math import sqrt
from math import atan2

class twister(Node):

    def __init__(self):
        super().__init__('twist')
        
        self.sub = self.create_subscription(String, 'pose', self.listener_callback, 10)
          
        #Valores que recibe del publisher path
        self.datax = [0.0, 0.0, 0.0, 0.0]
        self.datay = [0.0, 0.0, 0.0, 0.0]
        
        #Contador de: cantidad de valores recibidos, posicion array, contador tiempo        
        self.i = 0
        self.obj = 0
        self.contador = 0
        
        #Variables para girar 
        self.giro = False
        self.avanza = False
        self.calcular = True
        
        #Valores del twist
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        
        #Posicion actual del rbot y distancia con nueva coordenada
        self.posX = 0.0
        self.posY = 0.0
        self.posAng = 0.0 
        self.posDis = 0.0
        self.dis = 0.0  
        
        self.magnitud = 0.0
        self.producto = 0.0
        self.aang = 0.0 
        
        self.vectorX = 0.0
        self.vectorY = 0.0

        self.vector2X = 0.0
        self.vector2Y = 0.0
        
        #Publica al robot para moverse
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info('Twister node initialized!')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        #Verifica que haya recibido los 4 valores
        if(self.i > 3):
               
               if(self.calcular):
                     #Calcula la distancia entre puntos
                     self.dis = sqrt( pow(self.datax[self.obj] - self.posX,2) + pow(self.datay[self.obj] - self.posY,2) )
                     
                     print('Distancia = ')
                     print(self.dis)
               
                     #Calculo de angulo
                     self.aang = atan2(self.datay[self.obj] - self.posY, self.datax[self.obj] - self.posX)
                     print(self.aang)
                     
                     self.calcular = False
               
               if(self.giro):
                     self.angError = self.aang - self.posAng
                     if(self.angError > 0):
                           self.msg.angular.z = 0.1
                           self.posAng = self.posAng + 0.01
                           print(self.angError)
                     if(self.angError < 0):
                           self.msg.angular.z = -0.1
                           self.posAng = self.posAng - 0.01
                           print(self.angError)
                     
                     if(self.angError < 0.1 ) and (self.angError > -0.1):
                           print('Girar')
                           self.msg.angular.z = 0.0
                           self.giro = False
               elif(self.avanza):
                     self.disError = self.dis - self.posDis
                     if(self.disError > 0):
                           self.msg.linear.x = 0.8
                           self.posDis = self.posDis + 0.1
                           print(self.disError)
                     
                     if(self.disError < 0.0 ):
                           print('Avanza')
                           self.avanza = False
                           self.posDis = 0.0
                           self.msg.linear.x = 0.0
                           
                     
               else:
                     if (self.obj < 3):
                           #Establece la meta como posicion actual de la meta
                           self.posX = self.datax[self.obj]
                           self.posY = self.datay[self.obj]
                           
                           #Reinicia los contadores y mueve a la siguiente posicion deseada
                           self.obj = self.obj + 1
                           self.giro = True
                           self.avanza = True
                           self.contador = 0
                           self.calcular = True

                     else:
                           self.msg.linear.x = 0.0
                           self.msg.angular.z = 0.0
               
               self.publisher_.publish(self.msg)

    def listener_callback(self, msg):
        
        self.get_logger().info('Data: {}'.format(msg.data))
        mensaje = msg.data
        datos = mensaje.split(",")
        self.datax[self.i] = float(datos[0])
        self.datay[self.i] = float(datos[1])
        
        self.giro = True
        self.avanza = True
        self.i = self.i + 1
        
        print(self.i)
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
