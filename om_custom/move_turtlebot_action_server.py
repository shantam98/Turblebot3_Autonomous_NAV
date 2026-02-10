#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from om_custom.action import MoveTurtleBot
import math
import time


class MoveTurtleBotActionServer(Node):
	def __init__(self):
		super().__init__('move_turtlebot_action_server')

		self._action_server = ActionServer(
			self,
			MoveTurtleBot,
			'move_turtlebot',
			self.execute_callback
		)

		# Publishers and subscribers
		self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.odom_received = False

		self.sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

		self.get_logger().info('MoveTurtleBot Action Server is ready.')

	def pose_callback(self, msg):
		"""Callback for odometry topic"""
		self.robot_x = msg.pose.pose.position.x
		self.robot_y = msg.pose.pose.position.y
		self.odom_received = True

	def execute_callback(self, goal_handle):
		"""Execute the move turtlebot action"""

		lin_vel = goal_handle.request.linear_velocity
		ang_vel = goal_handle.request.angular_velocity
		distance = goal_handle.request.distance

		self.get_logger().info(f'Executing MoveTurtleBot goal with lin_vel={lin_vel}, ang_vel={ang_vel}, distance={distance}')
		# Wait for first odometry message
		while not self.odom_received:
			self.get_logger().info('Waiting for odometry...')
			time.sleep(0.1)
			rclpy.spin_once(self, timeout_sec=0.1)

		initial_x = self.robot_x
		initial_y = self.robot_y
		self.get_logger().info(f'Initial position: X={initial_x:.4f}, Y={initial_y:.4f}')

		feedback_msg = MoveTurtleBot.Feedback()
		vel = Twist()
		vel.linear.x = lin_vel
		vel.angular.z = ang_vel

		rate_period = 0.1  # 10 Hz

		while rclpy.ok():
			# Check if goal is canceled
			if goal_handle.is_cancel_requested:
				goal_handle.canceled()
				self.get_logger().info('Goal canceled')
				self.stop_robot()
				result = MoveTurtleBot.Result()
				result.success = False
				dist_traveled = math.sqrt(
					(self.robot_x - initial_x)**2 + (self.robot_y - initial_y)**2
				)
				result.distance_traveled = dist_traveled
				return result


			# Calculate distance traveled
			dist_traveled = math.sqrt(
				(self.robot_x - initial_x)**2 + (self.robot_y - initial_y)**2
			)

			# Check if destination reached
			if dist_traveled >= distance:
				self.stop_robot()
				self.get_logger().info(f'Destination reached! Traveled: {dist_traveled:.4f}m')
				break

			# Publish velocity
			self.pub.publish(vel)

			# Publish feedback
			# Publish feedback
			feedback_msg.current_x = self.robot_x
			feedback_msg.current_y = self.robot_y
			feedback_msg.distance_remaining = distance - dist_traveled
			goal_handle.publish_feedback(feedback_msg)


			time.sleep(rate_period)
			rclpy.spin_once(self, timeout_sec=0.0)

		# Set result
		goal_handle.succeed()
		result = MoveTurtleBot.Result()
		result.success = True
		result.distance_traveled = math.sqrt(
			(self.robot_x - initial_x)**2 + (self.robot_y - initial_y)**2
		)
		self.get_logger().info(f'Action succeeded. Distance traveled: {result.distance_traveled:.4f}m')
		return result

	def stop_robot(self):
		"""Publish zero velocity to stop the robot"""
		stop_msg = Twist()
		self.pub.publish(stop_msg)


def main(args=None):
	rclpy.init(args=args)
	node = MoveTurtleBotActionServer()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
