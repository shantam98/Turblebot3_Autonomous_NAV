#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from om_custom.action import MoveTurtleBot
import sys


class MoveTurtleBotActionClient(Node):
	def __init__(self):
		super().__init__('move_turtlebot_action_client')
		self._action_client = ActionClient(self, MoveTurtleBot, 'move_turtlebot')
		self._goal_handle = None

	def send_goal(self, linear_velocity, angular_velocity, distance):
		goal_msg = MoveTurtleBot.Goal()
		goal_msg.linear_velocity = linear_velocity
		goal_msg.angular_velocity = angular_velocity
		goal_msg.distance = distance

		self.get_logger().info(
			f'Sending goal: lin_vel={linear_velocity}, ang_vel={angular_velocity}, dist={distance}'
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
			f'Result: success={result.success}, distance_traveled={result.distance_traveled:.4f}m'
		)
		rclpy.shutdown()

	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info(
			f'Feedback: x={feedback.current_x:.4f}, y={feedback.current_y:.4f}, '
			f'remaining={feedback.distance_remaining:.4f}m'
		)



def main(args=None):
	if len(sys.argv) < 4:
		print("Usage: ros2 run om_custom move_turtlebot_client <linear_vel> <angular_vel> <distance>")
		sys.exit(1)

	lin_vel = float(sys.argv[1])
	ang_vel = float(sys.argv[2])
	distance = float(sys.argv[3])

	rclpy.init(args=args)
	client = MoveTurtleBotActionClient()
	client.send_goal(lin_vel, ang_vel, distance)

	try:
		rclpy.spin(client)
	except KeyboardInterrupt:
		pass
	finally:
		client.destroy_node()


if __name__ == '__main__':
	main()
