import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class My_Publisher(Node):

    def __init__(self):

        super().__init__('Odometry_node')
        self.publisher = self.create_publisher(Pose2D, 'odom', 0)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.wr_sub = self.create_subscription(Float32, "VelocityEncR", self.wr_callback, qos_profile)
        self.wl_sub = self.create_subscription(Float32, "VelocityEncL", self.wl_callback, qos_profile)

        self.get_logger().info('Odometry node initialized.')

        # Radio y distancia entre llantas
        self.r = 0.05
        self.l = 0.18

        # Datos que recibe de los encoders
        self.wr = 0
        self.wl = 0
        
        # Inicializar variables de posicion
        self.x_current = 0
        self.y_current = 0
        self.theta_current = 0

        self.pose_msg = Pose2D()

    def timer_callback(self):

        v = self.r * (self.wr + self.wl)/2
        w = self.r * (self.wr - self.wl)/self.l

        theta_next = self.theta_current + w * self.timer_period
        x_next = self.x_current + v * np.cos(theta_next) * self.timer_period
        y_next = self.y_current + v * np.sin(theta_next) * self.timer_period

        self.pose_msg.x = x_next
        self.pose_msg.y = y_next
        self.pose_msg.theta = theta_next

        self.publisher.publish(self.pose_msg)

        self.theta_current = theta_next
        self.x_current = x_next
        self.y_current = y_next

    def wr_callback(self, wr_msg):
        self.wr = wr_msg.data
        self.get_logger().info("VelocityEncR {}".format(wr_msg.data))

    def wl_callback(self, wl_msg):
        self.wl = wl_msg.data
        self.get_logger().info("VelocityEncL {}".format(wl_msg.data))

def main(args=None):

    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
