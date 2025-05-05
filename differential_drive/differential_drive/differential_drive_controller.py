import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from math import atan2, sqrt

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Publicador para el robot diferencial
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Buffer y TransformListener para TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer para calcular y publicar velocidades
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Nodo DifferentialDriveController iniciado.')

    def control_loop(self):
        """
        Obtiene la transformación entre el robot y la tortuga, y calcula el Twist deseado.
        """
        try:
            # Lee la transformación entre el robot (base_link) y la tortuga (turtle1)
            transform = self.tf_buffer.lookup_transform('base_link', 'turtle1', rclpy.time.Time())

            # Extrae las coordenadas relativas de la tortuga
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Calcula la distancia y el ángulo hacia la tortuga
            distance = sqrt(x**2 + y**2)
            angle_to_target = atan2(y, x)

            # Genera el comando de velocidad
            twist = Twist()
            twist.linear.x = 1.0 * distance  # Ganancia proporcional para velocidad lineal
            twist.angular.z = 2.5 * angle_to_target  # Ganancia proporcional para velocidad angular

            # Publica el comando de velocidad
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f'Comando enviado: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}')

        except Exception as e:
            self.get_logger().warn(f'No se pudo obtener la transformada: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
