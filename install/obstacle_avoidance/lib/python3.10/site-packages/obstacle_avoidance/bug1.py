import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


GOAL = (1.4185, -2.029)   # cambia según tu simulación

def euler_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return 0.0, 0.0, yaw  # solo nos interesa el yaw

def angle_to(x, y, gx, gy): return math.atan2(gy - y, gx - x)
def dist_to(x, y, gx, gy):  return math.hypot(gx - x, gy - y)
def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d
class Bug1(Node):
    def __init__(self):
        super().__init__('bug1')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x = self.y = self.yaw = 0.0
        self.state = 'GO_TO_GOAL'
        self.threshold = 0.5
        self.goal_tol  = 0.3
        self.hit_point = None       # dónde tocó el obstáculo
        self.min_dist  = float('inf')
        self.min_point = None       # punto más cercano al goal durante rodeo
        self.toured    = False      # ya completó el rodeo
        self.alejado   = False  # nuevo flag

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def fuera_del_mapa(self):
        return abs(self.x) > 3.0 or abs(self.y) > 3.0

    def scan_cb(self, msg):
        ranges = msg.ranges
        frente = [r for r in (list(ranges[0:30]) + list(ranges[330:360])) if not math.isinf(r)]
        min_frente = min(frente) if frente else float('inf')

        cmd = Twist()

        if self.fuera_del_mapa():
            cmd = Twist()
            cmd.linear.x = -0.2  # retrocede
            self.pub.publish(cmd)
            return

        if dist_to(self.x, self.y, *GOAL) < self.goal_tol:
            self.get_logger().info('Goal reached!')
            self.pub.publish(cmd)
            return

        if self.state == 'GO_TO_GOAL':
            if min_frente < self.threshold:
                self.hit_point = (self.x, self.y)
                self.min_dist  = dist_to(self.x, self.y, *GOAL)
                self.min_point = (self.x, self.y)
                self.toured    = False
                self.state     = 'FOLLOW_WALL'
            else:
                err = angle_diff(angle_to(self.x, self.y, *GOAL), self.yaw)
                cmd.linear.x  = 0.2
                cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))

        elif self.state == 'FOLLOW_WALL':
            d = dist_to(self.x, self.y, *GOAL)
            if d < self.min_dist:
                self.min_dist  = d
                self.min_point = (self.x, self.y)

            if not self.alejado:
                if dist_to(self.x, self.y, *self.hit_point) > 0.5:
                    self.alejado = True
            else:
                if dist_to(self.x, self.y, *self.hit_point) < 0.3:
                    self.toured = True

            if self.toured:
                self.state = 'RETURN_TO_MIN'
            else:
                # evasión activa — si hay obstáculo al frente, gira
                if min_frente < self.threshold:
                    izquierda = [r for r in list(ranges[30:90])   if not math.isinf(r)]
                    derecha   = [r for r in list(ranges[270:330]) if not math.isinf(r)]
                    min_iz = min(izquierda) if izquierda else float('inf')
                    min_der = min(derecha)  if derecha   else float('inf')
                    cmd.linear.x  = 0.0
                    cmd.angular.z = 1.0 if min_iz > min_der else -1.0
                else:
                    err = angle_diff(angle_to(self.x, self.y, *GOAL), self.yaw)
                    cmd.linear.x  = 0.15
                    cmd.angular.z = 0.5 if err > 0 else -0.5


        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Bug1()  
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()