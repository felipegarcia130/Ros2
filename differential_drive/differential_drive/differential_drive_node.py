import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
import math


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')

        # Inicializa las variables de posición global
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Tiempo inicial
        self.last_time = self.get_clock().now()

        # Broadcaster de TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscriptor al tópico /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer para publicar transformaciones
        self.timer = self.create_timer(0.1, self.publish_transform)

        self.get_logger().info('Nodo DifferentialDriveNode iniciado.')

    def cmd_vel_callback(self, msg):
        """
        Callback que procesa comandos de velocidad y actualiza la posición global del robot.
        """
        # Extrae las velocidades lineal y angular del mensaje Twist
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calcula el intervalo de tiempo desde la última actualización
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Convierte nanosegundos a segundos
        self.last_time = current_time

        # Actualiza la posición global del robot usando integración de Euler
        self.x += linear_x * math.cos(self.theta) * dt
        self.y += linear_x * math.sin(self.theta) * dt
        self.theta += angular_z * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normaliza theta

        self.get_logger().debug(f'Posición actualizada: x={self.x}, y={self.y}, theta={self.theta}')

    def publish_transform(self):
        """
        Publica la transformación del robot en el marco world.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f'Transformación publicada: x={self.x}, y={self.y}, theta={self.theta}')


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
