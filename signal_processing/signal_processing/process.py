import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalProcessor(Node):
    def __init__(self):
        super().__init__('process')

       
        self.subscription_signal = self.create_subscription(
            Float32, '/signal', self.process_signal, 10)

        self.subscription_time = self.create_subscription(
            Float32, '/time', self.process_time, 10)

        
        self.publisher = self.create_publisher(Float32, '/proc_signal', 10)

      
        self.timer = self.create_timer(0.1, self.publish_processed_signal)

        self.latest_processed_signal = None  
        self.current_time = 0.0  

    def process_signal(self, msg):
        raw_signal = msg.data  # Señal original recibida


        processed_signal = (raw_signal + 1) / 2  # Desplazar la señal para que sea positiva
        processed_signal *= 0.5  # Reducir la amplitud a la mitad
        processed_signal = math.sin(math.pi / 4 + processed_signal)  # Agregar fase

       
        self.latest_processed_signal = processed_signal

    def process_time(self, msg):
        self.current_time = msg.data  

    def publish_processed_signal(self):
        if self.latest_processed_signal is not None:
            msg_out = Float32()
            msg_out.data = self.latest_processed_signal
            self.publisher.publish(msg_out)

            # Mostrar en terminal
            self.get_logger().info(f'Time: {self.current_time:.2f}, Processed Signal: {self.latest_processed_signal:.2f}')

def main(args=None):
    rclpy.init(args=args)
    signal_processor = SignalProcessor()
    rclpy.spin(signal_processor)
    signal_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
