# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from custom_interfaces.srv import SetProcessBool
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Retrieve sine wave parameters
        self.declare_parameter('amplitude',2.0)
        self.declare_parameter('frequency', 0.5)
        self.declare_parameter('sample_time', 0.01)
        self.declare_parameter('input_type', 'square')

        self.amplitude = self.get_parameter('amplitude').value
        self.omega  =self.get_parameter('frequency').value
        self.timer_period = self.get_parameter('sample_time').value
        self.input_type = self.get_parameter('input_type').value
        
        self.system_running = False
        self.ctrl_running = False

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        #Create a service client for /EnableProcess
        self.cli = self.create_client(SetProcessBool, 'EnableProcess')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.cli2 = self.create_client(SetProcessBool, 'EnableCtrl')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Node Started \U0001F680")

        self.proc_send_request(True)
        self.ctrl_send_request(True)

        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):

        if not (self.system_running and self.ctrl_running):
            return  # Stop processing if simulation is not running
        
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate wave based on 'input_type' parameter
        if self.input_type == "sine":
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.input_type == "square":
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))


        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)
    
    def proc_send_request(self, enable: bool):
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
                self.system_running  = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running  = False
            self.get_logger().error(f'Service call failed: {e}')


    def ctrl_send_request(self, enable: bool):
        """Send a request to start or stop the simulation."""
        request2 = SetProcessBool.Request()
        request2.enable = enable

        future2 = self.cli2.call_async(request2)
        future2.add_done_callback(self.response2_callback)

    def response2_callback(self, future2):
        """Process the service response."""
        try:
            response = future2.result()
            if response.success:
                self.ctrl_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.ctrl_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.ctrl_running = False
            self.get_logger().error(f'Service call failed: {e}')


    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "amplitude":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid amplitude It cannot be negative.")
                    return SetParametersResult(successful=False, reason="amplitude cannot be negative")
                else:
                    self.amplitude = param.value  # Update internal variable
                    self.get_logger().info(f"gain updated to {self.amplitude}")

            if param.name == "frequency":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid frequency It cannot be negative.")
                    return SetParametersResult(successful=False, reason="frequencycannot be negative")
                else:
                    self.omega = param.value  # Update internal variable
                    self.get_logger().info(f"gain updated to {self.omega}")

            if param.name == "sample_time":
                    #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid sample_time It cannot be negative.")
                    return SetParametersResult(successful=False, reason="sample_time cannot be negative")
                else:
                    self.timer_period = param.value  # Update internal variable
                    self.get_logger().info(f"gain updated to {self.timer_period}")

            if param.name == "input_type":
                    #check if it is negative
                if (param.type_ != Parameter.Type.STRING):
                    self.get_logger().warn("Invalid parameter must be a string.")
                    return SetParametersResult(successful=False, reason="Invalid parameter must be a string.")

                new_type = param.value.lower()
                if new_type in ["sine", "square"]:
                    self.input_type = new_type
                    self.get_logger().info(f"✅ Input type updated: {self.input_type}")
                else:
                    self.get_logger().warn(f"❌ Invalid wave type '{new_type}'. Must be 'sine' or 'square'.")
                    return SetParametersResult(successful=False, reason="Invalid wave type.")

        return SetParametersResult(successful=True)

#Main
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

#Execute Node
if __name__ == '__main__':
    main()
