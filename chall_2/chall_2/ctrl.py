import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ControllerNode(Node):
    def __init__(self):
        super().__init__('ctrl')
        
          # Parámetros del controlador (pueden ajustarse desde un archivo YAML)
        self.declare_parameter('Kp', 2.0)  # Ganancia proporcional
        self.declare_parameter('Ki', 0.05)  # Ganancia integral
        self.declare_parameter('Kd',0.1)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        
        # Variables del controlador
        self.integral = 0.0
        self.last_error = 0.0  # Inicializamos error previo para cálculo de la derivada
        self.last_time = None   # Inicializamos last_time para evitar cálculos erróneos

        # Suscripción a los tópicos
        self.subscription_setpoint = self.create_subscription(
            Float32, '/set_point', self.setpoint_callback, 10)
        
        self.subscription_output = self.create_subscription(
            Float32, '/motor_speed_y', self.output_callback, 10)

        # Publicador de la señal de control
        self.publisher_control = self.create_publisher(Float32, '/motor_input_u', 10)

        # Variables para almacenar los valores más recientes
        self.setpoint = 0.0
        self.speed = 0.0

    def setpoint_callback(self, msg):
        """ Callback para actualizar el setpoint """
        self.setpoint = msg.data
        self.compute_control()

    def output_callback(self, msg):
        """ Callback para actualizar la salida del sistema """
        self.speed = msg.data
        self.compute_control()

    def compute_control(self):
        """ Implementación del Controlador PID """

        # Calcular el error
        error = self.setpoint - self.speed

        # Obtener el tiempo actual y calcular el delta de tiempo (dt)
        current_time = self.get_clock().now()

        if self.last_time is None:  # Primera iteración
            dt = 0.01  # Valor por defecto para evitar errores
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9  # Convertir nanosegundos a segundos
            if dt <= 0.0:
                dt = 0.01  # Evitar divisiones por cero
        
        self.last_time = current_time

        # **Cálculo del término derivativo**
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error  # Guardamos el error actual para la siguiente iteración

        # **Acumulación del término integral**
        self.integral += error * dt

        # **Cálculo de la señal de control PID**
        control_signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # **Limitar la salida para evitar valores extremos**
        control_signal = max(min(control_signal, 5.0), -5.0)

        # Publicar la señal de control
        msg = Float32()
        msg.data = control_signal
        self.publisher_control.publish(msg)

        # Log para depuración
        self.get_logger().info(f'Control: {control_signal:.3f} (Error: {error:.3f}, Integral: {self.integral:.3f}, Derivative: {derivative:.3f})')

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
