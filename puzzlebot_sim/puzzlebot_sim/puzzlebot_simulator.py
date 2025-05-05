import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros
import tkinter as tk
import time
import math
from matplotlib.figure import Figure
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from builtin_interfaces.msg import Time

# Parámetros físicos del robot
wheel_radius = 0.033  # radio de las ruedas
wheel_base = 0.160    # distancia entre ruedas
field_dims = (10, 10) # campo en metros

def inverse_kinematics(lin_vel, ang_vel):
    left_wheel_speed = (lin_vel - (wheel_base / 2) * ang_vel) / wheel_radius
    right_wheel_speed = (lin_vel + (wheel_base / 2) * ang_vel) / wheel_radius
    return left_wheel_speed, right_wheel_speed

def forward_kinematics(left_wheel_speed, right_wheel_speed):
    lin_vel = (left_wheel_speed + right_wheel_speed) / 2 * wheel_radius
    ang_vel = (right_wheel_speed - left_wheel_speed) * wheel_radius / wheel_base
    return lin_vel, ang_vel

class DifferentialKinematicsApp(Node):
    def __init__(self):
        super().__init__('tk_ros_simulator')

        # ROS 2: Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Estado inicial
        self.x, self.y, self.theta = 0.0,0.0,0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = time.time()

        # GUI
        self.root = tk.Tk()
        self.root.title("Puzzlebot Simulator - ROS2 + Tkinter")

        self.from_wheels = False
        self.use_forward = tk.BooleanVar(value=False)

        # Setup matplotlib
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        half_x = field_dims[0] / 2
        half_y = field_dims[1] / 2
        self.ax.set_xlim(-half_x, half_x)
        self.ax.set_ylim(-half_y, half_y)
        self.ax.set_title('Trazo')
        self.traj_line, = self.ax.plot([], [], 'b-', lw=1)
        self.triangle_patch = None
        self.trajectory = []

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Sliders
        frame = tk.Frame(self.root)
        frame.pack(side=tk.TOP, fill=tk.X)

        self.linear_speed_var = tk.DoubleVar(value=0.0)
        self.angular_speed_var = tk.DoubleVar(value=0.0)
        self.left_wheel_var = tk.DoubleVar(value=0.0)
        self.right_wheel_var = tk.DoubleVar(value=0.0)

        tk.Label(frame, text="Linear Speed").grid(row=0, column=0)
        tk.Scale(frame, variable=self.linear_speed_var, from_=-1.0, to=1.0, resolution=0.05,
                 orient=tk.HORIZONTAL, length=200).grid(row=0, column=1)

        tk.Label(frame, text="Angular Speed").grid(row=0, column=2)
        tk.Scale(frame, variable=self.angular_speed_var, from_=-2.0, to=2.0, resolution=0.05,
                 orient=tk.HORIZONTAL, length=200).grid(row=0, column=3)

        tk.Label(frame, text="Left Wheel").grid(row=1, column=0)
        tk.Scale(frame, variable=self.left_wheel_var, from_=-30.0, to=30.0, resolution=0.5,
                 orient=tk.HORIZONTAL, length=200, command=self.wheels_updated).grid(row=1, column=1)

        tk.Label(frame, text="Right Wheel").grid(row=1, column=2)
        tk.Scale(frame, variable=self.right_wheel_var, from_=-30.0, to=30.0, resolution=0.5,
                 orient=tk.HORIZONTAL, length=200, command=self.wheels_updated).grid(row=1, column=3)

        tk.Checkbutton(frame, text="Forward Mode", variable=self.use_forward).grid(row=2, column=0, sticky="w")

        self.update()

    def wheels_updated(self, event=None):
        if self.use_forward.get():
            left = self.left_wheel_var.get()
            right = self.right_wheel_var.get()
            lin, ang = forward_kinematics(left, right)
            self.linear_speed_var.set(lin)
            self.angular_speed_var.set(ang)
            self.from_wheels = True

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Leer velocidades desde sliders
        if self.use_forward.get():
            if not self.from_wheels:
                left = self.left_wheel_var.get()
                right = self.right_wheel_var.get()
                lin, ang = forward_kinematics(left, right)
                self.linear_speed_var.set(lin)
                self.angular_speed_var.set(ang)
            else:
                self.from_wheels = False
        else:
            lin = self.linear_speed_var.get()
            ang = self.angular_speed_var.get()
            left, right = inverse_kinematics(lin, ang)
            self.left_wheel_var.set(left)
            self.right_wheel_var.set(right)

        # Actualizar posición
        self.theta += ang * dt
        self.x += lin * math.cos(self.theta) * dt
        self.y += lin * math.sin(self.theta) * dt

        self.left_wheel_pos += left * dt
        self.right_wheel_pos += right * dt

        self.publish_ros_data(lin, ang)

        self.trajectory.append([self.x, self.y])
        if len(self.trajectory) > 1000:
            self.trajectory.pop(0)

        self.draw()
        self.root.after(50, self.update)

    def publish_ros_data(self, v_d, w_d):
        now_ros = self.get_clock().now().to_msg()

        # TF
        t = TransformStamped()
        t.header.stamp = now_ros
        t.header.frame_id = 'odom'
        t.child_frame_id = 'chassis'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(t)

        # Odometry
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'chassis'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = t.transform.rotation.z
        odom.pose.pose.orientation.w = t.transform.rotation.w
        odom.twist.twist.linear.x = v_d
        odom.twist.twist.angular.z = w_d
        self.odom_pub.publish(odom)

        # Joint States
        joint = JointState()
        joint.header.stamp = now_ros
        joint.name = ['left_wheel_joint', 'right_wheel_joint']
        joint.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_pub.publish(joint)

    def draw(self):
        if self.triangle_patch is not None:
            self.triangle_patch.remove()

        xs = [p[0] for p in self.trajectory]
        ys = [p[1] for p in self.trajectory]
        self.traj_line.set_data(xs, ys)

        # Dibuja robot
        local_triangle = [[0.3, 0], [-0.3, 0.2], [-0.3, -0.2]]
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        transformed = []
        for (x, y) in local_triangle:
            x_rot = x * cos_t - y * sin_t
            y_rot = x * sin_t + y * cos_t
            transformed.append([self.x + x_rot, self.y + y_rot])

        self.triangle_patch = patches.Polygon(transformed, closed=True, fc='k', ec='k')
        self.ax.add_patch(self.triangle_patch)
        self.canvas.draw_idle()

def main():
    rclpy.init()
    app_node = DifferentialKinematicsApp()
    app_node.root.mainloop()
    app_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
