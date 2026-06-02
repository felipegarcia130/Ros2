import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

N = 500

class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')
        # aquí va el código del mapa y partículas
        img = cv2.imread('/home/felipe/python/mapaarriba.png', cv2.IMREAD_GRAYSCALE)
        _, grid = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.grid = (grid == 255).astype(int)
        print(self.grid.sum())  # cuántos píxeles ocupados hay
        libres = np.argwhere(self.grid == 0)
        print(libres.shape)
        indices = np.random.choice(len(libres), N)
        self.particulas = libres[indices].astype(float)
        angulos = np.random.uniform(0, 2*np.pi, (N, 1))
        self.particulas = np.hstack([self.particulas, angulos])
        print(self.particulas.shape)
        mapa_vis = cv2.cvtColor((self.grid * 255).astype('uint8'), cv2.COLOR_GRAY2BGR)
        for p in self.particulas:
            cv2.circle(mapa_vis, (int(p[1]), int(p[0])), 2, (0, 0, 255), -1)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        ranges = msg.ranges
        self.get_logger().info(f'scan recibido: {len(ranges)} rayos')
        # aquí va el MCL

def main():
    rclpy.init()
    node = MCLNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()