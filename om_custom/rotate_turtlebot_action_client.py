#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from om_custom.action import RotateTurtleBot
import sys


class RotateTurtleBotActionClient(Node):
	def __init__(self):
		super().__init__('rotate_turtlebot_action_client')
		self._action_client = ActionClient(self, RotateTurtleBot, 'rotate_turtlebot')
		self._goal_handle = None

	def send_goal(self, angular_velocity, theta):
		goal_msg = RotateTurtleBot.Goal()
		goal_msg.angular_velocity = angular_velocity
		goal_msg.theta = theta

		self.get_logger().info(
			f'Sending goal: ang_vel={angular_velocity}, theta={theta:.4f} rad'
		)

		self._action_client.wait_for_server()
		self._send_goal_future = self._action_client.send_goal_async(
			goal_msg, feedback_callback=self.feedback_callback
		)
		self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return

		self.get_logger().info('Goal accepted')
		self._goal_handle = goal_handle
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info(
			f'Result: success={result.success}, '
			f'final_z={result.final_orientation_z:.4f}, '
			f'final_w={result.final_orientation_w:.4f}'
		)
		rclpy.shutdown()

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(
			f'Feedback: z={feedback.current_orientation_z:.4f}, '
			f'w={feedback.current_orientation_w:.4f}, '
			f'error={feedback.orientation_error:.4f}'
		)


def main(args=None):
	if len(sys.argv) < 3:
		print("Usage: ros2 run om_custom rotate_turtlebot_client <angular_vel> <theta_radians>")
		sys.exit(1)

	ang_vel = float(sys.argv[1])
	theta = float(sys.argv[2])

	rclpy.init(args=args)
	client = RotateTurtleBotActionClient()
	client.send_goal(ang_vel, theta)

	try:
		rclpy.spin(client)
	except KeyboardInterrupt:
		pass
	finally:
		client.destroy_node()


if __name__ == '__main__':
	main()
