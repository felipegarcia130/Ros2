# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32


#Class Definition
class SignalProcessing(Node):
    def __init__(self):
        super().__init__('signal_proc')

        self.signal_subscription = self.create_subscription(Float32, 'signal', self.signal_callback,10)
        self.time_subscription = self.create_subscription(Float32, 'time', self.time_callback,10)
        self.publisher = self.create_publisher(Float32, 'proc_signal', 10)

        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb) 
        self.start_time = self.get_clock().now()

        self.angle = np.pi/2
        self.offset = 1.0
        self.amplitude = 0.5

        self.signal_msg = Float32()
        self.time_msg = Float32()
        self.proc_msg =Float32()
        
    #Timer Callback
    def timer_cb(self):

        if ((self.wrap_to_Pi(self.time_msg.data) >= -np.pi/2) and (self.wrap_to_Pi(self.time_msg.data)<= np.pi/2)):
            signal = self.signal_msg.data * np.cos(self.angle) + np.sqrt(1-np.power((self.signal_msg.data),2)) * np.sin(self.angle) 
        
        else:
            signal = self.signal_msg.data * np.cos(self.angle) - np.sqrt(1-np.power((self.signal_msg.data),2)) * np.sin(self.angle)

        self.proc_msg.data = self.amplitude * (signal + self.offset)
        self.publisher.publish(self.proc_msg)

    #Subscriber Callback
    def signal_callback(self, signal_in):
        self.signal_msg = signal_in

    #Subscriber Callback
    def time_callback(self, time_in):
        self.time_msg = time_in

    # Wrap to Pi function
    def wrap_to_Pi(self,theta):
        result = np.fmod((theta+np.pi),(2*np.pi))
        if (result<0):
            result += 2 * np.pi
        return result - np.pi

#Main
def main(args=None):
    rclpy.init(args=args)

    signal_process = SignalProcessing()

    try:
        rclpy.spin(signal_process)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        signal_process.destroy_node()

#Execute Node
if __name__ == '__main__':
    main()
