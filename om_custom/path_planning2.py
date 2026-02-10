#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

waypoints = [
    (-1.75, 0.0),
     (0.0, -1.75),
     (1.75, 0.0),
    (0.0, 1.75),
     (-1.75, 0.0)
]


def yaw_to_quaternion(yaw):
    qw = math.cos(yaw / 2.0)
    qz = math.sin(yaw / 2.0)
    return qz, qw


class PathPlanning2Node(Node):
    def __init__(self):
        super().__init__('path_planning2')
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # odom state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_qz = 0.0
        self.current_qw = 1.0
        self.odom_received = False

        # obstacle state
        self.obstacle_detected = False
        self.obstacle_angle = 0.0

        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.get_logger().info('PathPlanning2Node ready.')

    # ------------------------------------------------------------------ odom
    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_qz = msg.pose.pose.orientation.z
        self.current_qw = msg.pose.pose.orientation.w
        self.odom_received = True

    # ------------------------------------------------------------------ scan
    def _scan_cb(self, msg):
        front = msg.ranges[0] if msg.ranges[0] != float('inf') else 10.0
        left = msg.ranges[45] if msg.ranges[45] != float('inf') else 10.0
        right = msg.ranges[315] if msg.ranges[315] != float('inf') else 10.0

        threshold = 0.5
        if front < threshold or left < threshold or right < threshold:
            self.obstacle_detected = True
            if left < right:
                self.obstacle_angle = -1.0   # obstacle on left -> rotate right
            else:
                self.obstacle_angle = 1.0    # obstacle on right -> rotate left
        else:
            self.obstacle_detected = False
            self.obstacle_angle = 0.0

    # ------------------------------------------------------------------ stop
    def stop_robot(self):
        self._cmd_pub.publish(Twist())

    # ---------------------------------------------------------- move_distance
    def move_distance(self, linear_velocity, distance):
        """
        Drive forward `distance` metres at `linear_velocity`.
        Returns True if completed, False if interrupted by obstacle.
        """
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        initial_x = self.robot_x
        initial_y = self.robot_y

        vel = Twist()
        vel.linear.x = float(linear_velocity)

        while rclpy.ok():
            # obstacle check
            if self.obstacle_detected:
                self.stop_robot()
                dist = math.hypot(self.robot_x - initial_x, self.robot_y - initial_y)
                self.get_logger().info(f'Move interrupted by obstacle after {dist:.4f}m')
                return False

            dist_traveled = math.hypot(self.robot_x - initial_x, self.robot_y - initial_y)
            if dist_traveled >= distance:
                self.stop_robot()
                self.get_logger().info(f'Move complete. Traveled {dist_traveled:.4f}m')
                return True

            self._cmd_pub.publish(vel)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.stop_robot()
        return False

    # ----------------------------------------------------------- rotate_to_yaw
    def rotate_to_yaw(self, angular_velocity, theta, tolerance=0.01):
        """
        Rotate to absolute yaw `theta` (rad) using quaternion error.
        """
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        desired_qz, desired_qw = yaw_to_quaternion(theta)

        # pick direction based on current yaw vs desired
        current_yaw = math.atan2(2.0 * self.current_qw * self.current_qz,
                                 self.current_qw ** 2 - self.current_qz ** 2)
        ang_vel = -abs(angular_velocity) if current_yaw >= theta else abs(angular_velocity)

        self.get_logger().info(
            f'Rotating to yaw={theta:.4f} rad, ang_vel={ang_vel:.4f}'
        )

        vel = Twist()
        vel.angular.z = float(ang_vel)

        while rclpy.ok():
            orient_error = math.sqrt(
                (self.current_qz - desired_qz) ** 2 +
                (self.current_qw - desired_qw) ** 2
            )
            if orient_error <= tolerance:
                self.stop_robot()
                self.get_logger().info(f'Rotation complete. Error: {orient_error:.4f}')
                return True

            self._cmd_pub.publish(vel)
            time.sleep(0.025)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.stop_robot()
        return False

    # -------------------------------------------------------- obstacle avoidance
    def perform_avoidance(self, linear_speed=0.1):
        if not self.obstacle_detected:
            return

        ang = self.obstacle_angle
        twist = Twist()
        twist.angular.z = ang

        self.get_logger().info(f'Avoidance: rotating with ang_vel={ang}')
        while rclpy.ok() and self.obstacle_detected:
            self._cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        rclpy.spin_once(self, timeout_sec=0.05)

        self.obstacle_detected = False
        self.get_logger().info('Avoidance: moving forward 0.1m')
        self.move_distance(linear_speed, 0.1)

    # -------------------------------------------------------- waypoint execution
    def execute_waypoints(self, wps, linear_speed=0.1, angular_speed=0.1):
        if not wps:
            self.get_logger().info('No waypoints provided')
            return

        while not self.odom_received:
            self.get_logger().info('Waiting for odometry...')
            rclpy.spin_once(self, timeout_sec=0.1)

        for i, (tx, ty) in enumerate(wps):
            while True:
                dx = tx - self.robot_x
                dy = ty - self.robot_y
                distance = math.hypot(dx, dy)
                if distance < 0.05:
                    self.get_logger().info(f'Waypoint {i+1} reached: x={tx:.3f}, y={ty:.3f}')
                    break

                theta = math.atan2(dy, dx)
                self.get_logger().info(
                    f'Waypoint {i+1}: rotate to {theta:.4f} rad, then move {distance:.4f} m'
                )

                # rotate to heading
                rot_ok = self.rotate_to_yaw(angular_speed, theta)
                if not rot_ok:
                    self.get_logger().warn('Rotate failed; skipping waypoint')
                    break

                # move towards waypoint
                move_ok = self.move_distance(linear_speed, distance)

                # obstacle interrupted -> avoid and retry same waypoint
                if not move_ok and self.obstacle_detected:
                    self.get_logger().info('Obstacle interrupted move -> performing avoidance')
                    self.perform_avoidance(linear_speed=linear_speed)
                    self.get_logger().info('Avoidance done -> re-navigating to waypoint')
                    continue

                break


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning2Node()
    try:
        node.execute_waypoints(waypoints, linear_speed=0.25, angular_speed=0.3)
    except KeyboardInterrupt:
        node.get_logger().info('Path planning interrupted')
    finally:
        node.stop_robot()
        node.get_logger().info('Shutting down path_planning2 node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
