import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TurtlePoseTransformNode(Node):
    def __init__(self):
        super().__init__('turtle_pose_transform_node')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Nodo TurtlePoseTransformNode iniciado.")

    def pose_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'turtle1'

        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0

        theta = msg.theta
        transform.transform.rotation.z = math.sin(theta / 2.0)
        transform.transform.rotation.w = math.cos(theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f'Transformación publicada: x={msg.x}, y={msg.y}, theta={theta}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TurtlePoseTransformNode(Node):
    def __init__(self):
        super().__init__('turtle_pose_transform_node')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Nodo TurtlePoseTransformNode iniciado.")

    def pose_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'turtle1'

        transform.transform.translation.x = msg.x
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0

        theta = msg.theta
        transform.transform.rotation.z = math.sin(theta / 2.0)
        transform.transform.rotation.w = math.cos(theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f'Transformación publicada: x={msg.x}, y={msg.y}, theta={theta}')


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
