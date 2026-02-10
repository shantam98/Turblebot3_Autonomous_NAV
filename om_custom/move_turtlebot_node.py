#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveTurtleBotNode(Node):
    def __init__(self):
        super().__init__('move_turtlebot_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info('MoveTurtleBotNode ready.')

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.odom_received = True

    def stop_robot(self):
        self.pub.publish(Twist())

    def move(self, linear_velocity, angular_velocity, distance, obstacle_check_fn=None):
        """
        Drive the robot at the given velocities until the Euclidean distance
        from the starting position reaches `distance`.

        obstacle_check_fn: optional callable returning True if an obstacle is
        detected. If it returns True the move is cancelled early and this
        method returns False.

        Returns True if the full distance was covered, False if interrupted.
        """
        # wait for odom
        while not self.odom_received:
            self.get_logger().info('Waiting for odometry...')
            rclpy.spin_once(self, timeout_sec=0.1)

        initial_x = self.robot_x
        initial_y = self.robot_y

        vel = Twist()
        vel.linear.x = float(linear_velocity)
        vel.angular.z = float(angular_velocity)

        rate_period = 0.1  # 10 Hz

        while rclpy.ok():
            # check for obstacle
            if obstacle_check_fn is not None and obstacle_check_fn():
                self.stop_robot()
                dist = math.hypot(self.robot_x - initial_x, self.robot_y - initial_y)
                self.get_logger().info(f'Move interrupted by obstacle after {dist:.4f}m')
                return False

            dist_traveled = math.hypot(self.robot_x - initial_x, self.robot_y - initial_y)
            if dist_traveled >= distance:
                self.stop_robot()
                self.get_logger().info(f'Move complete. Traveled {dist_traveled:.4f}m')
                return True

            self.pub.publish(vel)
            time.sleep(rate_period)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.stop_robot()
        return False


def main(args=None):
    """Standalone test: ros2 run om_custom move_turtlebot_node <lin_vel> <ang_vel> <distance>"""
    import sys
    if len(sys.argv) < 4:
        print('Usage: move_turtlebot_node <linear_vel> <angular_vel> <distance>')
        sys.exit(1)

    lin_vel = float(sys.argv[1])
    ang_vel = float(sys.argv[2])
    distance = float(sys.argv[3])

    rclpy.init(args=args)
    node = MoveTurtleBotNode()
    try:
        node.move(lin_vel, ang_vel, distance)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
