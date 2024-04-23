import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class My_Publisher(Node):

    def __init__(self):

        super().__init__('Controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 0)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Controller node initialized.')
        self.msg = Twist()

        self.elapsed_time = 0.0
        self.max_time_move = 4.0
        self.max_time_turn = 1.2
        self.is_moving = True  # Indica si el robot está moviéndose o girando
        self.start_turn_time = None  # Almacena el tiempo de inicio del giro

    def timer_callback(self):

        if self.is_moving:

            self.msg.linear.x = 0.2 if self.elapsed_time < self.max_time_move else 0.0
            self.msg.angular.z = 0.0

            if self.elapsed_time >= self.max_time_move:

                self.is_moving = False
                self.start_turn_time = time.time()  # Almacenamos el tiempo de inicio del giro
                self.elapsed_time = 0.0
        else:

            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.5 if time.time() - self.start_turn_time < self.max_time_turn else 0.0

            if time.time() - self.start_turn_time >= self.max_time_turn:

                self.is_moving = True
                self.elapsed_time = 0.0

        self.publisher.publish(self.msg)
        self.elapsed_time += self.timer_period

def main(args=None):

    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
