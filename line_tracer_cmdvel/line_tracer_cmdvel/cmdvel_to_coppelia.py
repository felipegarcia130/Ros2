import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class CmdVelToCoppelia(Node):
    def __init__(self):
        super().__init__('cmdvel_to_coppelia')
        self.subscription=self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        client=RemoteAPIClient()
        self.sim=client.getObject('sim')

        self.robot=self.sim.getObject('/LineTracer')
        self.left_motor = self.sim.getObject('/LineTracer/DynamicLeftJoint')
        self.right_motor = self.sim.getObject('/LineTracer/DynamicRightJoint')

        self.sim.setObjectPosition(self.robot, -1, [0, 0, 0.1])  
        self.sim.setObjectOrientation(self.robot,-1, [0, 0, 0]) 

        self.wheel_radius=0.033  # radio de las ruedas
        self.base_length=0.160  # distancia entre ruedas

        self.get_logger().info('CoppeliaSim client connected and ready to receive /cmd_vel.')

    def listener_callback(self, msg):
        v=msg.linear.x
        w=msg.angular.z

        v_left=(v-(self.base_length/2)*w)/self.wheel_radius
        v_right=(v+(self.base_length/2)*w)/self.wheel_radius

        self.sim.setJointTargetVelocity(self.left_motor,v_left)
        self.sim.setJointTargetVelocity(self.right_motor,v_right)

def main(args=None):
    rclpy.init(args=args)
    node=CmdVelToCoppelia()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if main() == '__main__':
    main()        