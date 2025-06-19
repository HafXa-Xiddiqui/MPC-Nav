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

        # Dynamically updated obstacle positions
        self.obstacle_poses = {
            'cylinder_obstacle': None,
            'cylinder_obstacle_1': None,
            'cylinder_obstacle_2': None
        }

        # Subscribe to obstacle odometry
        self.create_subscription(Odometry, '/cylinder_obstacle/odom',
                                 self.make_obstacle_callback('cylinder_obstacle'), 10)
        self.create_subscription(Odometry, '/cylinder_obstacle_1/odom',
                                 self.make_obstacle_callback('cylinder_obstacle_1'), 10)
        self.create_subscription(Odometry, '/cylinder_obstacle_2/odom',
                                 self.make_obstacle_callback('cylinder_obstacle_2'), 10)

    def robot_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.target_pose = msg

    def make_obstacle_callback(self, name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.obstacle_poses[name] = [x, y]
        return callback

    def control_loop(self):
        if self.robot_pose is None or self.target_pose is None:
            return

        # Extract robot pose
        rx = self.robot_pose.pose.pose.position.x
        ry = self.robot_pose.pose.pose.position.y
        rtheta = self.get_yaw(self.robot_pose.pose.pose.orientation)

        # Extract target pose
        tx = self.target_pose.pose.pose.position.x
        ty = self.target_pose.pose.pose.position.y

        # Get obstacle positions or default far away
        obstacles = []
        for key in ['cylinder_obstacle', 'cylinder_obstacle_1', 'cylinder_obstacle_2']:
            pos = self.obstacle_poses[key]
            if pos is not None:
                obstacles.append(pos)
            else:
                obstacles.append([10.0, 10.0])  # Far dummy obstacle

        # Run MPC
        v, omega = run_mpc([rx, ry, rtheta], [tx, ty], obstacles)

        # Publish velocity
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def get_yaw(self, q):
        from tf_transformations import euler_from_quaternion
        orientation = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(orientation)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = MPCFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
