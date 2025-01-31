# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool


#Class Definition
class PIDController(Node):
    def __init__(self):
        super().__init__('controller')

        # Declare parameters
        # Controller Gains
        self.declare_parameter('gain_Kp', 0.9)
        self.declare_parameter('gain_Ki', 0.5)
        self.declare_parameter('gain_Kd', 0.0)
        self.declare_parameter('sample_time', 0.01)

        # DC Motor Parameters
        self.sample_time = self.get_parameter('sample_time').value
        self.gain_Kp = self.get_parameter('gain_Kp').value
        self.gain_Ki = self.get_parameter('gain_Ki').value
        self.gain_Kd = self.get_parameter('gain_Kd').value

        #Set the messages
        self.ctrl_output_msg = Float32()
        self.ctrl_input_msg = Float32()

        #Set variables to be used
        self.set_point = 0.0
        self.angular_vel = 0.0
        self.prev_vel = 0.0
        self.error_int = 0.0
        self.output_u = 0.0
        self.ctrl_running = False 
    
        #Declare publishers, subscribers and timers
        self.ctrl_sp_sub = self.create_subscription(Float32, 'set_point', self.sp_callback,10)
        self.ctrl_vel_sub = self.create_subscription(Float32, 'motor_speed_y', self.vel_callback,10)
        self.ctrl_output_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        self.timer = self.create_timer(self.sample_time, self.timer_cb) 

        #Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        #Set Server callback
        self.srv = self.create_service(SetProcessBool, 'EnableCtrl', self.ctrl_service_callback)

        #Node Started
        self.get_logger().info('Controller Node Started \U0001F680')   
        
    #Timer Callback
    def timer_cb(self):
        
        if not self.ctrl_running:
            return  # Stop processing if simulation is not running         
        
        #Controller
        error = self.set_point - self.angular_vel
        #PI controller
        self.error_int += (error) * self.sample_time
        
        u  = self.gain_Kp * error + self.gain_Ki * self.error_int + self.gain_Kd * (self.angular_vel - self.prev_vel)/self.sample_time
        self.prev_vel = self.angular_vel
        
        #Publish the result
        self.ctrl_output_msg.data = u
        self.ctrl_output_pub.publish(self.ctrl_output_msg)

    #Subscriber Callback
    def sp_callback(self, sp_sgn):
        self.set_point = sp_sgn.data

    #Subscriber Callback
    def vel_callback(self, vel_sgn):
        self.angular_vel = vel_sgn.data


    # Service Callback to Start/Stop Simulation
    def ctrl_service_callback(self, request, response):
        if request.enable:
            self.ctrl_running = True
            self.get_logger().info("🚀 Controller Started")
            response.success = True
            response.message = "Controller Started Successfully"
        else:
            self.ctrl_running = False
            self.get_logger().info("🛑 Controller Stopped")
            response.success = True
            response.message = "Controller Stoped Successfully"

        return response
    

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "gain_Kp":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid gain! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="gain cannot be negative")
                else:
                    self.gain_Kp = param.value  # Update internal variable
                    self.get_logger().info(f"gain updated to {self.gain_Kp}")

            if param.name == "gain_Ki":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid gain! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="gain cannot be negative")
                else:
                    self.gain_Ki = param.value  # Update internal variable
                    self.get_logger().info(f"gain updated to {self.gain_Ki}")

                if param.name == "gain_Kd":
                    #check if it is negative
                    if (param.value < 0.0):
                        self.get_logger().warn("Invalid gain! It cannot be negative.")
                        return SetParametersResult(successful=False, reason="gain cannot be negative")
                    else:
                        self.gain_Kd = param.value  # Update internal variable
                        self.get_logger().info(f"gain updated to {self.gain_Kd}")

                if param.name == "sample_time":
                    #check if it is negative
                    if (param.value < 0.0):
                        self.get_logger().warn("Invalid sample_time It cannot be negative.")
                        return SetParametersResult(successful=False, reason="sample_time cannot be negative")
                    else:
                        self.sample_time = param.value  # Update internal variable
                        self.get_logger().info(f"gain updated to {self.sample_time}")
                
        return SetParametersResult(successful=True)
        

#Main
def main(args=None):
    rclpy.init(args=args)

    node = PIDController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
