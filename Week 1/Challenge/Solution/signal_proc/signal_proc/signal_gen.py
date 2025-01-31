# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32


#Class Definition
class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_gen')
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.start_time = self.get_clock().now()

        self.signal_msg = Float32()
        self.time_msg = Float32()
        
    #Timer Callback
    def timer_cb(self):

        elapsed_time = self.get_clock().now() - self.start_time
        self.time_msg.data = elapsed_time.nanoseconds/1e9

        self.signal_msg.data = np.sin(self.time_msg.data)

        self.signal_publisher.publish(self.signal_msg)
        self.time_publisher.publish(self.time_msg)

        self.get_logger().info('Publising: "%f"' % self.signal_msg.data)

#Main
def main(args=None):
    rclpy.init(args=args)

    signal_publisher  = SignalGenerator()

    try:
        rclpy.spin(signal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        signal_publisher.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()
