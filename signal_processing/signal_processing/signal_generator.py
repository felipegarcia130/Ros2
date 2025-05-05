import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')

        self.signal_publisher = self.create_publisher(Float32, '/signal' , 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)

        self.timer = self.create_timer(0.1, self.publish_signal)

        self.start_time = time.time()

    def publish_signal(self):
        t = time.time() - self.start_time #tiempo
        signal = math.sin(t) #señal

        msg_signal = Float32()
        msg_signal.data = signal

        self.signal_publisher.publish(msg_signal)

        msg_time = Float32()
        msg_time.data = t
        self.time_publisher.publish(msg_time)

        #Para mostrar en consola
        self.get_logger().info(f'Time: {t:.2f}, Signal: {signal: .2f}')

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()


