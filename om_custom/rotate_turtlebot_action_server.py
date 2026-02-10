#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from om_custom.action import RotateTurtleBot
import math
import time


def yaw_to_quaternion(yaw):
	"""
	Convert a yaw angle (radians) to quaternion components (z, w).
	For 2D motion, roll = 0 and pitch = 0, so:
		qw = cos(yaw / 2)
		qz = sin(yaw / 2)
	Returns (qz, qw).
	"""
	qw = math.cos(yaw / 2.0)
	qz = math.sin(yaw / 2.0)
	return qz, qw


class RotateTurtleBotActionServer(Node):
	def __init__(self):
		super().__init__('rotate_turtlebot_action_server')

		self._action_server = ActionServer(
			self,
			RotateTurtleBot,
			'rotate_turtlebot',
			self.execute_callback
		)

		# Publishers and subscribers
		self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.current_qz = 0.0
		self.current_qw = 1.0
		self.odom_received = False

		self.sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

		self.get_logger().info('RotateTurtleBot Action Server is ready.')

	def pose_callback(self, msg):
		"""Callback for odometry topic"""
		self.current_qz = msg.pose.pose.orientation.z
		self.current_qw = msg.pose.pose.orientation.w
		self.odom_received = True

	def execute_callback(self, goal_handle):
		"""Execute the rotate turtlebot action"""
		

		ang_vel = goal_handle.request.angular_velocity
		theta = goal_handle.request.theta
		tolerance = 0.01

		self.get_logger().info(f'Executing RotateTurtleBot goal with angular_velocity={ang_vel}, theta={theta}')
		# Compute desired quaternion from yaw
		desired_qz, desired_qw = yaw_to_quaternion(theta)
		self.get_logger().info(
			f'Desired yaw: {theta:.4f} rad -> Quaternion: z={desired_qz:.4f}, w={desired_qw:.4f}'
		)

		# Wait for first odometry message
		while not self.odom_received:
			self.get_logger().info('Waiting for odometry...')
			time.sleep(0.1)
			rclpy.spin_once(self, timeout_sec=0.1)

		self.get_logger().info(
			f'Initial orientation: z={self.current_qz:.4f}, w={self.current_qw:.4f}'
		)

		feedback_msg = RotateTurtleBot.Feedback()
		vel = Twist()
		vel.angular.z = ang_vel

		rate_period = 25e-3  # 40 Hz

		while rclpy.ok():
			# Check if goal is canceled
			if goal_handle.is_cancel_requested:
				goal_handle.canceled()
				self.get_logger().info('Goal canceled')
				self.stop_robot()
				result = RotateTurtleBot.Result()
				result.success = False
				result.final_orientation_z = self.current_qz
				result.final_orientation_w = self.current_qw
				return result

			# Compute Euclidean distance between current and desired orientation (z, w)
			orient_error = math.sqrt(
				(self.current_qz - desired_qz)**2 +
				(self.current_qw - desired_qw)**2
			)

			# Check if desired orientation is reached
			if orient_error <= tolerance:
				self.stop_robot()
				self.get_logger().info(
					f'Desired orientation reached! Error: {orient_error:.4f}'
				)
				break

			# Publish angular velocity
			self.pub.publish(vel)

			# Publish feedback
			feedback_msg.current_orientation_z = self.current_qz
			feedback_msg.current_orientation_w = self.current_qw
			feedback_msg.orientation_error = orient_error
			goal_handle.publish_feedback(feedback_msg)

			time.sleep(rate_period)
			rclpy.spin_once(self, timeout_sec=0.0)

		# Set result
		goal_handle.succeed()
		result = RotateTurtleBot.Result()
		result.success = True
		result.final_orientation_z = self.current_qz
		result.final_orientation_w = self.current_qw
		self.get_logger().info(
			f'Action succeeded. Final orientation: z={self.current_qz:.4f}, w={self.current_qw:.4f}'
		)
		return result

	def stop_robot(self):
		"""Publish zero velocity to stop the robot"""
		stop_msg = Twist()
		self.pub.publish(stop_msg)


def main(args=None):
	rclpy.init(args=args)
	node = RotateTurtleBotActionServer()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
