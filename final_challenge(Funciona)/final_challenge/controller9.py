import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import rclpy.duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from std_msgs.msg import Float32

from math import sqrt
from math import atan2
from math import pi

class controller_node(Node):

    def __init__(self):

        super().__init__('control_node_turning')
        #Valores que determinan si se llego al objetivo
        self.recibido = False
        self.alto = False

        #Mensaje de señal y linea
        self.senal = ''

        #Contadores secuencia interseccion
        self.contador = 0.0
        self.rotondaTrue = False
        self.signCooldown = 2.0
        self.interCooldown = 1.0
        self.interMidCooldown = 10.0
        self.finCooldown = 3.0
        self.altoCooldown = 2.0
        self.altoUsadoCooldown = 11.0

        #Condiciones booleanas
        self.interseccion = False
        self.segInterseccion = False
        self.terInterseccion = False
        self.detectError = False
        self.altoUsado = False
        self.slowUsado = False

        self.secuencia = False

        #Valores de semaforo
        self.prevSign = 'green_light'

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
        self.kp_angular = Float32()
        self.kp_angular = 0.0002

        self.err = Float32()
        self.err = 0.0

        #Valores del encoder
        self.encL = 0.0
        self.encR = 0.0
        
        #Subscripiones de path, encoder izquierdo y derecho
        qos_profile = rclpy.qos.qos_profile_sensor_data
        # qos_profile = QoSProfile(lifespan = rclpy.duration.Duration(seconds = 5))
        self.odom_sub = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile)
        self.error_sub = self.create_subscription(Float32, 'error', self.error_callback, 10)

        self.signs_sub = self.create_subscription(String, 'predict_signs', self.signs_callback, qos_profile)        
        self.semaforo_sub = self.create_subscription(String, 'predict_lights', self.lights_callback, qos_profile)

        self.interseccion_sub = self.create_subscription(String, 'interseccion', self.interseccion_callback, 10)

        #Crea los publisher a la velocidad linear y angular
        self.lock_publisher = self.create_publisher(String, 'lock', 10)
        self.puzzlebot_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.done_publisher = self.create_publisher(Twist, 'signal', 10)
        self.get_logger().info('NODE NUDDLES activaded!')       

        #Llama a publicar cada 0.1 segundos
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #Mensaje para publisher
        self.msg = String()

#--------------------------------------------------Funcion principal--------------------------------------
    def timer_callback(self):
        #Comprueba si la señal es slow
        if(self.senal == 'slow' and self.slowUsado == False):

            self.kp_angular = 0.0001
            self.kp_linear = 0.1
            self.get_logger().info('Slow Effect iniciada!')

            self.interMidCooldown -= self.timer_period
            print(self.interMidCooldown)

            if self.interMidCooldown <= 2.0:
                self.kp_angular = 0.0002
                self.kp_linear = 0.2
                self.slowUsado = True
                self.interMidCooldown = 10

                self.twist_msg.linear.x = self.kp_linear * 0.3
                self.twist_msg.angular.z = self.kp_angular * self.err

        #Comprueba si se encuentra en una interseccion
        if(self.interseccion == True):
            self.get_logger().info('Interglobalion detectada!') 
            self.signCooldown = self.signCooldown - 0.1
            self.slowUsado = False

            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0 * self.err

            #Espera 2 segundos para detectar señal
            if(self.signCooldown < 0.0):
                self.get_logger().info('Iniciando enfriamento de señales!') 
                #Establecer valor


                #Gira a la izquierda
                if (self.senal == 'Turn_Left'):
                    self.get_logger().info('Girando a la left!') 
                    #Avanza para dejar de leer la señal
                    self.interCooldown = self.interCooldown - 0.1
                    self.kp_linear = 0.1 
                    self.err = 0.0

                    #Si ya se cruzo la interseccion
                    if(self.interCooldown < 0.0):
                        self.get_logger().info('Ya se cruzo la primera interseccion!') 
                        #Avanzar constantemente
                        self.kp_linear = 0.2
                        self.err = 0.0

                        #Si detecta una segunda interseccion gira constantemente a la izquierda
                        if(self.segInterseccion == True or self.detectError == True):
                            self.get_logger().info('Se detecto la segunda interseccion!') 
                            self.kp_linear = 0
                            self.err = 200

                            self.interMidCooldown = self.interMidCooldown - 0.1
                            if(self.interMidCooldown < 2.0):
                                self.get_logger().info('El cooldown se agoto!') 

                                self.finCooldown = self.finCooldown - 0.1
                                self.kp_linear = 0.2 
                                self.err = 0.0

                                if(self.finCooldown < 0.0):
                                    self.get_logger().info('El ciclo del agua ya se completo!') 
                                    #Termina la secuencia de interseccion y reinicia todos los valores
                                    self.signCooldown = 2.0
                                    self.interCooldown = 0.5
                                    self.interseccion = False
                                    self.segInterseccion = False
                                    self.terInterseccion = False
                                    self.detectError = False
                                    self.interMidCooldown = 11.0
                                    self.finCooldown = 3.0

                    #Establecer valor
                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)
                    
                    #self.altoUsado = False

                #Gira a la derecha
                if (self.senal == 'Turn_Right'):
                    self.get_logger().info('Girando a la dereright!') 

                    #Avanza para dejar de leer la señal
                    self.interCooldown = self.interCooldown - 0.1
                    self.kp_linear = 0.1 
                    self.err = 0.0

                    #Si ya se cruzo la interseccion
                    if(self.interCooldown < 0.0):
                        self.get_logger().info('Ya se cruzo la primera interseccion!') 
                        #Avanzar constantemente
                        self.kp_linear = 0.1
                        self.err = 0.0

                        #Si detecta una segunda interseccion gira constantemente a la izquierda
                        if(self.segInterseccion == True or self.detectError == True):
                            self.get_logger().info('Se detecto la segunda interseccion!') 
                            self.kp_linear = 0
                            self.err = -200

                            self.interMidCooldown = self.interMidCooldown - 0.1
                            if(self.interMidCooldown < 0.0):
                                self.get_logger().info('El cooldown se agoto!') 

                                self.finCooldown = self.finCooldown - 0.1
                                self.kp_linear = 0.1 
                                self.err = 0.0

                                if(self.finCooldown < 0.0):
                                    self.get_logger().info('El ciclo del agua ya se completo!') 
                                    #Termina la secuencia de interseccion y reinicia todos los valores
                                    self.signCooldown = 2.0
                                    self.interCooldown = 0.5
                                    self.interseccion = False
                                    self.segInterseccion = False
                                    self.terInterseccion = False
                                    self.detectError = False
                                    self.interMidCooldown = 10.0
                                    self.finCooldown = 3.0

                    #Establecer valor
                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)

                    #self.altoUsado = False

                #Avanza derecho en la interseccion
                if (self.senal == 'Forward'):
                    self.get_logger().info('Pisale a fondo!') 

                    #Establecer valor
                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)

                    #self.altoUsado = False

                #Rotonda
                if (self.senal == 'Rotonda'):
                    self.get_logger().info('Toco darle la vuelta al mundo!') 

                    if(self.rotondaTrue == False):
                        #Avanza para dejar de leer la señal
                        self.interCooldown = self.interCooldown - 0.1
                        self.kp_linear = 0.1 
                        self.err = 0.0

                        #Si ya se cruzo la interseccion
                        if(self.interCooldown < 0.0):
                            self.get_logger().info('Ya se cruzo la primera interseccion!') 
                            #Avanzar constantemente
                            self.kp_linear = 0.1
                            self.err = 0.0

                            #Si detecta una segunda interseccion gira constantemente a la izquierda
                            if(self.segInterseccion == True or self.detectError == True):
                                self.get_logger().info('Se detecto la segunda interseccion!') 
                                self.kp_linear = 0
                                self.err = -200

                                self.interMidCooldown = self.interMidCooldown - 0.1
                                if(self.interMidCooldown < 2.0):
                                    self.get_logger().info('El cooldown se agoto!') 

                                    self.finCooldown = self.finCooldown - 0.1
                                    self.kp_linear = 0.1 
                                    self.err = 0.0

                                    if(self.finCooldown < 0.0):
                                        self.get_logger().info('El ciclo del agua ya se completo!') 
                                        #Termina la secuencia de interseccion y reinicia todos los valores
                                        self.signCooldown = 2.0
                                        self.interCooldown = 0.5
                                        self.interseccion = False
                                        self.segInterseccion = False
                                        self.terInterseccion = False
                                        self.detectError = False
                                        self.interMidCooldown = 10.0
                                        self.finCooldown = 3.0
                                        self.rotondaTrue = True
                                        self.kp_linear = 0.2 
                    else:
                        self.get_logger().info('Segunda etapa rotonda!') 
                        
                    #Establecer valor
                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)

                    #self.altoUsado = False
                    
                #Avanza derecho en la interseccion
                if (self.senal == 'Alto' and self.altoUsado == False):
                    self.get_logger().info('Parele o atropella al chamaco!') 

                    self.altoCooldown = self.altoCooldown - 0.1
                    self.kp_linear = 0.0
                    self.kp_angular = 0.0                        

                    self.get_logger().info('Se encontro un signo de alto!') 
                    if(self.altoCooldown < 0.0):
                        self.altoUsado = True
                        self.altoCooldown = 2.0
                        self.get_logger().info('Hasta la proxima alto!')
                        self.kp_linear = 0.0
                        self.kp_angular = 0.0

                    #Establecer valor
                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)

                if (self.senal == 'Alto' and self.altoUsado == True):
                    self.kp_linear = 0.2
                    self.kp_angular = 0.0
                    self.altoUsadoCooldown = self.altoUsadoCooldown - 0.1

                    if(self.altoUsadoCooldown < 0.0):
                        self.altoUsadoCooldown = 10.0
                        self.interseccion = False
                        self.altoUsado = False
                        self.kp_angular = 0.0002

                    self.twist_msg.linear.x = self.kp_linear * 0.3
                    self.twist_msg.angular.z = self.kp_angular * self.err
                    self.puzzlebot_publisher.publish(self.twist_msg)


            self.puzzlebot_publisher.publish(self.twist_msg)

        else:
            #Establecer valor
            self.twist_msg.linear.x = self.kp_linear * 0.3
            self.twist_msg.angular.z = self.kp_angular * self.err
            self.puzzlebot_publisher.publish(self.twist_msg)

#---------------------------------------------------------------------------------------------------------

    #Recibe datos del odometria
    def odom_callback(self, msg):

        self.pose_msg.x = msg.x
        self.pose_msg.y = msg.y
        self.pose_msg.theta = msg.theta
    
    #Recibe el error de la camara
    def error_callback(self, msg):

        self.err = msg.data
        
        #Si se detecta error durante interseccion se vuelve canon
        if(self.interCooldown < 0.0):
            self.detectError = True

    #Llama a la función si detecto una interseccion
    def interseccion_callback(self, msg):
        if(self.interMidCooldown < 0.0):
            self.get_logger().info('Si lees este mensaje es porque ya es de dia ya!') 
            self.terInterseccion = True

        if(self.interCooldown > 0.0):
            self.interseccion = True
            self.msg.data = 'lock'
            self.lock_publisher.publish(self.msg)
        else:
            self.segInterseccion == True        

    #Recibe el color del semaforo
    def lights_callback(self, msg):

        self.get_logger().info(msg.data)
        mensaje = msg.data
        
        if (mensaje == 'green_light' and self.prevSign == 'red_light'):
            self.kp_angular = 0.0002
            self.kp_linear = 0.2
            self.prevSign = 'green_light'

        if (mensaje == 'yellow_light'):
            self.kp_angular = 0.0001
            self.kp_linear = 0.1
            self.prevSign = 'yellow_light'

        if (mensaje == 'red_light' and self.senal != 'Alto' and self.prevSign == 'yellow_light'):
            self.kp_angular = 0.0
            self.kp_linear = 0.0
            self.prevSign = 'red_light'
            

    #Recibe la señal de trafico
    def signs_callback(self, msg):

        self.get_logger().info(msg.data)
        mensaje = msg.data
        self.senal = mensaje




def main(args=None):

    rclpy.init(args=args)
    c_n = controller_node()
    rclpy.spin(c_n)
    c_n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()