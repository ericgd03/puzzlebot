#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

class My_Publisher(Node):
    def __init__(self):
        super().__init__('talker_node')


        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.publisher = self.create_publisher(Pose2D, 'point', qos_profile)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!')
        self.msg = Pose2D()

    def timer_callback(self):
        mensaje = input("Introduce coordenadas (valor,valor):")
        datos = mensaje.split(",")
        self.msg.x = float(datos[0])
        self.msg.y = float(datos[1])
        
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()