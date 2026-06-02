import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

GOAL = (1.4185, -2.029)  # cambia según tu simulación

def angle_to(x, y, gx, gy): return math.atan2(gy - y, gx - x)
def dist_to(x, y, gx, gy):  return math.hypot(gx - x, gy - y)
def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d
class Bug0(Node):
    def __init__(self):
        super().__init__('bug0')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = self.y = self.yaw = 0.0
        self.state = 'GO_TO_GOAL'   # estados: GO_TO_GOAL, FOLLOW_WALL
        self.threshold = 0.5
        self.goal_tol  = 0.3

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_cb(self, msg):
        ranges = msg.ranges
        frente = [r for r in (list(ranges[0:30]) + list(ranges[330:360])) if not math.isinf(r)]
        min_frente = min(frente) if frente else float('inf')

        cmd = Twist()

        if dist_to(self.x, self.y, *GOAL) < self.goal_tol:
            self.get_logger().info('Goal reached!')
            self.pub.publish(cmd)
            return

        if self.state == 'GO_TO_GOAL':
            if min_frente < self.threshold:
                self.state = 'FOLLOW_WALL'
            else:
                # girar hacia el goal y avanzar
                err = angle_diff(angle_to(self.x, self.y, *GOAL), self.yaw)
                cmd.linear.x  = 0.2
                cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))

        elif self.state == 'FOLLOW_WALL':
            # si puede ir directo al goal, volver
            if min_frente > self.threshold:
                err = angle_diff(angle_to(self.x, self.y, *GOAL), self.yaw)
                if abs(err) < 0.3:
                    self.state = 'GO_TO_GOAL'
            cmd.linear.x  = 0.15
            cmd.angular.z = 0.5   # sigue el borde girando izquierda

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Bug0()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()