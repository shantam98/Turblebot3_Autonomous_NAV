#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw):
    """Convert yaw (rad) to quaternion (qz, qw)."""
    qw = math.cos(yaw / 2.0)
    qz = math.sin(yaw / 2.0)
    return qz, qw


class RotateTurtleBotNode(Node):
    def __init__(self):
        super().__init__('rotate_turtlebot_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_qz = 0.0
        self.current_qw = 1.0
        self.odom_received = False
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info('RotateTurtleBotNode ready.')

    def _odom_cb(self, msg):
        self.current_qz = msg.pose.pose.orientation.z
        self.current_qw = msg.pose.pose.orientation.w
        self.odom_received = True

    def stop_robot(self):
        self.pub.publish(Twist())

    def rotate(self, angular_velocity, theta, tolerance=0.01):
        """
        Rotate the robot to the absolute yaw `theta` (rad).
        Uses quaternion error just like the action server.

        Returns True on success, False if interrupted.
        """
        # wait for odom
        while not self.odom_received:
            self.get_logger().info('Waiting for odometry...')
            rclpy.spin_once(self, timeout_sec=0.1)

        desired_qz, desired_qw = yaw_to_quaternion(theta)
        self.get_logger().info(
            f'Rotating to yaw={theta:.4f} rad (qz={desired_qz:.4f}, qw={desired_qw:.4f}), '
            f'ang_vel={angular_velocity}'
        )

        vel = Twist()
        vel.angular.z = float(angular_velocity)
        rate_period = 25e-3  # 40 Hz

        while rclpy.ok():
            orient_error = math.sqrt(
                (self.current_qz - desired_qz) ** 2 +
                (self.current_qw - desired_qw) ** 2
            )
            if orient_error <= tolerance:
                self.stop_robot()
                self.get_logger().info(f'Rotation complete. Error: {orient_error:.4f}')
                return True

            self.pub.publish(vel)
            time.sleep(rate_period)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.stop_robot()
        return False


def main(args=None):
    """Standalone test: ros2 run om_custom rotate_turtlebot_node <angular_vel> <theta>"""
    import sys
    if len(sys.argv) < 3:
        print('Usage: rotate_turtlebot_node <angular_vel> <theta_rad>')
        sys.exit(1)

    ang_vel = float(sys.argv[1])
    theta = float(sys.argv[2])

    rclpy.init(args=args)
    node = RotateTurtleBotNode()
    try:
        node.rotate(ang_vel, theta)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
