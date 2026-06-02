import rclpy, math, heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import numpy as np

GOAL = (1.249, -1.421)

def angle_to(x, y, gx, gy): return math.atan2(gy - y, gx - x)
def dist_to(x, y, gx, gy):  return math.hypot(gx - x, gy - y)
def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d

def astar(grid, width, height, start, goal):
    def h(a, b): return math.hypot(b[0]-a[0], b[1]-a[1])
    def neighbors(x, y):
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]
        result = []
        for dx, dy in dirs:
            nx, ny = x+dx, y+dy
            if 0 <= nx < width and 0 <= ny < height:
                if grid[ny, nx] == 0:
                    cost = 1.414 if dx != 0 and dy != 0 else 1.0
                    result.append((nx, ny, cost))
        return result

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score   = {start: 0}
    f_score   = {start: h(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        for nx, ny, cost in neighbors(*current):
            neighbor = (nx, ny)
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor]   = tentative_g
                f_score[neighbor]   = tentative_g + h(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []


class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_node')
        self.sub_map  = self.create_subscription(OccupancyGrid, '/map',      self.map_cb,  10)
        self.sub_pose = self.create_subscription(PoseStamped,   '/mcl_pose', self.pose_cb, 10)
        self.sub_scan = self.create_subscription(LaserScan,     '/scan',     self.scan_cb, 10)
        self.pub      = self.create_publisher(Twist, '/cmd_vel', 10)

        self.grid      = None
        self.width     = 0
        self.height    = 0
        self.res_x     = 0.0
        self.res_y     = 0.0
        self.origin_x  = 0.0
        self.origin_y  = 0.0

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0

        self.min_front_range = float('inf')  # obstacle check

        self.waypoints    = []
        self.current_wp   = 0
        self.path_ready   = False
        self.map_received = False
        self.create_timer(0.1, self.control_loop)

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Rayos frontales ±20° (índices 160-200 de 360 rayos)
        front = ranges[160:200]
        front = front[np.isfinite(front) & (front > 0.05)]
        self.min_front_range = float(np.min(front)) if len(front) > 0 else float('inf')

    def map_cb(self, msg):
        if self.map_received:
            return

        self.width    = msg.info.width
        self.height   = msg.info.height
        self.res_x    = msg.info.resolution
        self.res_y    = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))

        self.map_received = True
        self.get_logger().info(
            f'Mapa recibido: {self.width}x{self.height} '
            f'res={self.res_x:.5f} origin=({self.origin_x:.3f},{self.origin_y:.3f})'
        )
        self.compute_path()

    def pose_cb(self, msg):
        self.robot_x   = msg.pose.position.x
        self.robot_y   = msg.pose.position.y
        q = msg.pose.orientation
        self.robot_yaw = 2 * math.atan2(q.z, q.w)

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.res_x)
        gy = int((wy - self.origin_y) / self.res_y)
        gy = self.height - 1 - gy
        return gx, gy

    def grid_to_world(self, gx, gy):
        gy_real = self.height - 1 - gy
        wx = gx     * self.res_x + self.origin_x + self.res_x / 2
        wy = gy_real * self.res_y + self.origin_y + self.res_y / 2
        return wx, wy

    def cell_value(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx]
        return 'FUERA'

    def nearest_free(self, gx, gy):
        for r in range(1, 30):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = gx+dx, gy+dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.grid[ny, nx] == 0:
                            return nx, ny
        return gx, gy

    def compute_path(self):
        if self.grid is None:
            return

        start = self.world_to_grid(self.robot_x, self.robot_y)
        start = self.nearest_free(*start)
        goal  = self.world_to_grid(*GOAL)
        goal  = self.nearest_free(*goal)

        self.get_logger().info(f'Robot world: ({self.robot_x:.3f},{self.robot_y:.3f})')
        self.get_logger().info(f'Start celda: {start} valor: {self.cell_value(*start)}')
        self.get_logger().info(f'Goal  celda: {goal}  valor: {self.cell_value(*goal)}')

        path = astar(self.grid, self.width, self.height, start, goal)

        if not path:
            self.get_logger().warn('A* no encontró ruta')
            return

        self.waypoints  = [self.grid_to_world(gx, gy) for gx, gy in path[::5]]
        self.current_wp = 0
        self.path_ready = True
        self.get_logger().info(f'Ruta lista: {len(self.waypoints)} waypoints')

    def control_loop(self):
        if not self.path_ready:
            return

        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('¡Goal alcanzado!')
            self.pub.publish(Twist())
            self.path_ready = False
            return

        # Obstacle check con láser directo
        if self.min_front_range < 0.35:
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.5
            self.pub.publish(cmd)
            # Saltar al siguiente waypoint para no quedarse atascado
            self.current_wp += 1
            return

        wx, wy = self.waypoints[self.current_wp]
        cmd    = Twist()

        if dist_to(self.robot_x, self.robot_y, wx, wy) < 0.3:
            self.current_wp += 1
            return

        err = angle_diff(angle_to(self.robot_x, self.robot_y, wx, wy), self.robot_yaw)

        if abs(err) > 0.5:
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))
        else:
            cmd.linear.x  = 0.2
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()