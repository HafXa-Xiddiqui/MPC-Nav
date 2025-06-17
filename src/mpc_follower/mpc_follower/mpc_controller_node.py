import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np
from mpc_follower.mpc_solver import run_mpc

class MPCFollower(Node):
    def __init__(self):
        super().__init__('mpc_follower')
        self.robot_pose = None
        self.target_pose = None

        self.create_subscription(Odometry, '/odom', self.robot_callback, 10)
        self.create_subscription(Odometry, '/target_sphere/odom', self.target_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.control_loop)

        # ✅ Known obstacle positions (same as in your launch file)
        self.obstacles = [
            [0.8, 0.5],
            [-0.6, 0.8],
            [0.0, -1.0]
        ]

    def robot_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.target_pose = msg

    def control_loop(self):
        if self.robot_pose is None or self.target_pose is None:
            return

        # Extract robot state
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        rtheta = self.get_yaw(self.robot_pose.pose.pose.orientation)

        # Extract target position
        tx = self.target_pose.pose.pose.position.x
        ty = self.target_pose.pose.pose.position.y

        # ✅ Call MPC with obstacle positions
        v, omega = run_mpc([rx, ry, rtheta], [tx, ty], self.obstacles)

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def get_yaw(self, q):
        import math
        from tf_transformations import euler_from_quaternion
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = MPCFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
