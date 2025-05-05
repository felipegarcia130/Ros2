import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import yaml
import os

class GazeboSimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # Parámetros configurables
        self.declare_parameter('path_file', '')
        self.declare_parameter('move_duration', 3.0)
        self.declare_parameter('loop_path', False)

        path_file = self.get_parameter('path_file').get_parameter_value().string_value
        self.move_duration = self.get_parameter('move_duration').get_parameter_value().double_value
        self.loop_path = self.get_parameter('loop_path').get_parameter_value().bool_value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.update)

        self.path = self.load_path(path_file)
        self.current_index = 0
        self.state = 'MOVING'
        self.start_time = time.time()
        self.current_position = [0.0, 0.0]
        self.target_position = self.path[0]

        self.get_logger().info('✅ Nodo de simulación iniciado y listo.')

    def load_path(self, file_path):
        full_path = os.path.expanduser(file_path)
        if not os.path.exists(full_path):
            self.get_logger().error(f'❌ Archivo de trayectoria no encontrado: {full_path}')
            return []

        with open(full_path, 'r') as f:
            data = yaml.safe_load(f)
            return [[p['x'], p['y']] for p in data['path']]

    def update(self):
        if not self.path or self.state != 'MOVING':
            return

        now = time.time()
        t = min((now - self.start_time) / self.move_duration, 1.0)

        start = self.current_position
        end = self.target_position

        new_x = start[0] + (end[0] - start[0]) * t
        new_y = start[1] + (end[1] - start[1]) * t
        self.current_position = [new_x, new_y]

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        speed = distance / self.move_duration

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        if t >= 1.0:
            self.get_logger().info(f'✅ Punto {self.current_index + 1} alcanzado')
            self.current_position = end
            self.current_index += 1
            if self.current_index >= len(self.path):
                if self.loop_path:
                    self.current_index = 0
                else:
                    self.state = 'IDLE'
                    self.publisher.publish(Twist())  # Detener el robot
                    self.get_logger().info('🏁 Trayectoria completada.')
                    return

            self.target_position = self.path[self.current_index]
            self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = GazeboSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
