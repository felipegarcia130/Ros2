import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmdvel_test_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_cmd)
        self.stage = 0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def publish_cmd(self):
        msg = Twist()
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        if elapsed < 3:
            # Etapa 1: Avanzar recto
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.get_logger().info('Avanzando...')
        elif elapsed < 6:
            # Etapa 2: Girar a la izquierda
            msg.linear.x = 0.0
            msg.angular.z = math.pi/8  # Girar 90 grados
            self.get_logger().info('Girando...')
        
        elif elapsed < 9:
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.get_logger().info('Avanzando...')    
        else:
            # Etapa 3: Detenerse
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Detenido.')
            self.publisher.publish(msg)
            self.destroy_timer(self.timer)
            return

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
