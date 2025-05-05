# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from custom_interfaces.srv import SetProcessBool

# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('sp_gen')  # Se asegura de que el nodo se llame 'sp_gen'

        # **Definir amplitud y frecuencia**
        self.amplitude = 2.0  # Altura de la onda cuadrada
        self.omega = 1.0  # Frecuencia de la onda cuadrada
        self.system_running = False

        # **Publicador en /set_point**
        self.signal_publisher = self.create_publisher(Float32, '/set_point', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # Crear cliente de servicio
        self.cli = self.create_client(SetProcessBool, 'EnableProcess')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Crear mensaje y variables
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started 🚀")

        self.send_request(True)

    # Timer Callback: Generar y Publicar Onda Cuadrada
    def timer_cb(self):
        if not self.system_running:
            return  # Stop processing if simulation is not running
        
        # Calcular el tiempo transcurrido
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # **Generar onda cuadrada**
        self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))

        # Publicar la señal
        self.signal_publisher.publish(self.signal_msg)
        self.get_logger().info(f'Publishing setpoint: {self.signal_msg.data:.3f}')

    def send_request(self, enable: bool):
        """Send a request to start or stop the simulation."""
        request = SetProcessBool.Request()
        request.enable = enable

        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Process the service response."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.system_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running = False
            self.get_logger().error(f'Service call failed: {e}')

# Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()
