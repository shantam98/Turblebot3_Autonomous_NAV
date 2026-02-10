#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from om_custom.action import MoveTurtleBot, RotateTurtleBot
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

waypoints = [
    (-1.75, 0.0),
     (0.0, -1.75),
     (1.75, 0.0),
    (0.0, 1.75),
     (-1.75, 0.0)
]

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')
        self._move_client = ActionClient(self, MoveTurtleBot, 'move_turtlebot')
        self._rotate_client = ActionClient(self, RotateTurtleBot, 'rotate_turtlebot')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_received = False
        # cmd_vel publisher to stop/rotate during avoidance
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # laser subscriber for obstacle detection
        self.obstacle_detected = False
        self.obstacle_angle = 0.0
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def wait_for_servers(self, timeout_sec=None):
        self.get_logger().info('Waiting for action servers...')
        self._move_client.wait_for_server(timeout_sec=timeout_sec)
        self._rotate_client.wait_for_server(timeout_sec=timeout_sec)
        self.get_logger().info('Action servers available')

    def send_move(self, linear_velocity: float, angular_velocity: float, distance: float):
        goal_msg = MoveTurtleBot.Goal()
        goal_msg.linear_velocity = float(linear_velocity)
        goal_msg.angular_velocity = float(angular_velocity)
        goal_msg.distance = float(distance)

        self.get_logger().info(
            f'Sending move goal: lin_vel={linear_velocity}, ang_vel={angular_velocity}, dist={distance}'
        )

        send_goal_future = self._move_client.send_goal_async(goal_msg)
        # poll until goal is accepted/rejected
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Move goal rejected')
            return None

        get_result_future = goal_handle.get_result_async()

        # Monitor the active goal, cancel if obstacle detected
        while rclpy.ok() and not get_result_future.done():
            if self.obstacle_detected:
                self.get_logger().warn('Obstacle detected -> canceling current move goal')
                cancel_future = goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    rclpy.spin_once(self, timeout_sec=0.05)
                # stop robot immediately
                self.get_logger().info('Stopping robot due to obstacle')
                self._cmd_pub.publish(Twist())
                rclpy.spin_once(self, timeout_sec=0.05)
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        # wait for the result to be ready
        # bot stopped at 
        

        if self.obstacle_detected:
            self.get_logger().info('Bot stopped at : {:.4f}, {:.4f} due to obstacle'.format(self.robot_x, self.robot_y))
            return None
        
        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        result = get_result_future.result().result
        self.get_logger().info(
            f'Move result: success={result.success}, distance_traveled={result.distance_traveled:.4f}m'
        )
        return result

    def send_rotate(self, angular_velocity: float, theta: float):
        goal_msg = RotateTurtleBot.Goal()
        goal_msg.angular_velocity = float(angular_velocity)
        goal_msg.theta = float(theta)

        self.get_logger().info(
            f'Sending rotate goal: ang_vel={angular_velocity}, theta={theta:.4f} rad'
        )

        send_goal_future = self._rotate_client.send_goal_async(goal_msg)
        # poll until goal is accepted/rejected
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Rotate goal rejected')
            return None

        get_result_future = goal_handle.get_result_async()
        # poll until rotation completes
        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        result = get_result_future.result().result
        self.get_logger().info(
            f'Rotate result: success={result.success}, final_z={result.final_orientation_z:.4f}'
        )
        return result

    def _scan_cb(self, msg: LaserScan):
        # Check 3 points: front (0), left (60), right (300)
        front = msg.ranges[0] if msg.ranges[0] != float('inf') else 10.0
        left = msg.ranges[60] if msg.ranges[60] != float('inf') else 10.0
        right = msg.ranges[300] if msg.ranges[300] != float('inf') else 10.0
        threshold = 0.3
        if front < threshold or left < threshold or right < threshold:
            self.obstacle_detected = True
            # rotate away from the closer side
            if left < right:
                self.obstacle_angle = -1.0   # obstacle on left -> rotate right
            else:
                self.obstacle_angle = 1.0  # obstacle on right -> rotate left
            self.get_logger().info('Obstacle detected! front: {:.2f}, left: {:.2f}, right: {:.2f} at {:.4f}, {:.4f}'.format(front, left, right, self.robot_x, self.robot_y))
        else:
            self.obstacle_detected = False
            self.obstacle_angle = 0.0

    def _perform_avoidance(self, linear_speed=0.1):
        # Called when obstacle detected. Rotate until obstacle clears, then move forward 0.1 m
        if not self.obstacle_detected:
            return

        # choose rotation direction: if obstacle_angle > 0 (left), rotate right (negative), else rotate left
        ang = self.obstacle_angle
        twist = Twist()
        twist.angular.z = ang
        
        # rotate until no obstacle in FOV
        self.get_logger().info(f'Performing rotation to avoid obstacle, ang_vel={ang}')
        while rclpy.ok() and self.obstacle_detected:
            self._cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # rotate the bot by pi/4 to fully clear obstacle
        self.get_logger().info('Stopping rotation')

        # stop rotation
        self._cmd_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

        # small forward bump using move action (may itself detect obstacles)
        self.obstacle_detected = False  # reset flag before move
        self.get_logger().info('Avoidance: moving forward 0.1m')
        self.send_move(linear_speed, 0.0, 0.1)

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w
        self.odom_received = True

    def execute_waypoints(self, waypoints, linear_speed=0.2, angular_speed=0.15):
        """
        waypoints: list of (x,y) tuples. For each waypoint:
          - compute dx,dy
          - compute desired absolute yaw with atan2(dy,dx)
          - call rotate action to that yaw
          - compute euclidean distance and call move action with that distance
        """
        if not waypoints:
            self.get_logger().info('No waypoints provided')
            return

        # wait for odom
        while not self.odom_received:
            self.get_logger().info('Waiting for odometry...')
            rclpy.spin_once(self, timeout_sec=0.1)

        for i, (tx, ty) in enumerate(waypoints):
            while True:
                # recompute vector from current position to target
                dx = tx - self.robot_x
                dy = ty - self.robot_y
                distance = math.hypot(dx, dy)
                if distance < 0.05:
                    self.get_logger().info(f'Waypoint {i+1} reached: x={tx:.3f}, y={ty:.3f}')
                    break

                # desired absolute yaw
                theta = math.atan2(dy, dx)
                self.get_logger().info(f'Waypoint {i+1}: rotate to {theta:.4f} rad, then move {distance:.4f} m')
                # rotate to heading
                # if theta is negative, rotate clockwise (negative ang vel) and convert theta to positive
                # current yaw from quaternion
                current_yaw = math.atan2(2.0 * (self.qw * self.qz),  (self.qw**2 - self.qz**2))
                self.get_logger().info(f'Current yaw: {current_yaw:.4f} rad')
                if current_yaw>=theta:
                    self.get_logger().info('Rotating clockwise')
                    rot_res = self.send_rotate(-1*angular_speed,theta)
                else:
                    self.get_logger().info('Rotating counter-clockwise')
                    rot_res = self.send_rotate(angular_speed, theta)
                if rot_res is None or not getattr(rot_res, 'success', True):
                    self.get_logger().warn('Rotate failed or rejected; skipping waypoint')
                    break

                # move towards waypoint
                mv_res = self.send_move(linear_speed, 0.0, distance)

                # if obstacle was hit, perform avoidance then loop back to re-aim at same waypoint
                if self.obstacle_detected:
                    self.get_logger().info('Obstacle interrupted move -> performing avoidance')
                    self._perform_avoidance(linear_speed=linear_speed)
                    self.get_logger().info('Avoidance done -> re-navigating to waypoint')
                    continue

                # move completed normally, waypoint reached
                break

    def execute_square(self, side_length=1.0, linear_speed=0.2, angular_speed=0.15):
        # Drive a square: move forward, rotate 90 degrees, repeat 4 times
        for i in range(4):
            self.get_logger().info(f'Starting side {i+1} of square')
            self.send_move(linear_speed, 0.0, side_length)
            # rotate 90 degrees = pi/2
            m = (i+1) % 4
            self.send_rotate(angular_speed, m*math.pi / 2)

    def execute_custom_path(self, path):
        # path: list of dicts with keys: type: 'move'|'rotate' and params
        for step in path:
            if step.get('type') == 'move':
                params = step.get('params', {})
                self.send_move(params.get('linear_velocity', 0.2), params.get('angular_velocity', 0.0), params.get('distance', 1.0))
            elif step.get('type') == 'rotate':
                params = step.get('params', {})
                self.send_rotate(params.get('angular_velocity', 0.15), params.get('theta', math.pi/2))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()

    try:
        node.wait_for_servers()
        # default behavior: square path
        node.execute_waypoints(waypoints, linear_speed=0.1, angular_speed=0.1)

    except KeyboardInterrupt:
        node.get_logger().info('Path planning interrupted')
    finally:
        node.get_logger().info('Shutting down path_planning node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
