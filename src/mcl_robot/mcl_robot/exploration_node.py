import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class CovarianzaOdom(Node):
    def __init__(self):
        super().__init__('covarianza_odom')
        # Parámetros del robot
        self.r = 0.051   # radio de rueda (m)
        self.b = 0.18    # separación entre ruedas (m)

        # Constantes de error (calibrar después)
        self.k_r = 0.1
        self.k_l = 0.1
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        

def main(args=None):
    rclpy.init(args=args)
    node = CovarianzaOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()