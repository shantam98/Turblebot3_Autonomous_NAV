#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class OpenManipulatorController(Node):

    def __init__(self):
        super().__init__('openmanipulator_controller')

        # Arm action client
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

    def move_arm(self, joint_positions, time_sec=3.0):
        """
        joint_positions: [j1, j2, j3, j4] in radians
        """

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(time_sec)
        point.time_from_start.nanosec = int((time_sec % 1) * 1e9)

        goal.trajectory.points.append(point)

        self.arm_client.wait_for_server()
        self.get_logger().info('Sending arm goal...')
        return self.arm_client.send_goal_async(goal)

    def move_gripper(self, position, time_sec=1.0):
        """
        position: 
          0.0   -> fully open
          ~0.01 -> closed (depends on your gripper)
        """

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(time_sec)
        point.time_from_start.nanosec = int((time_sec % 1) * 1e9)

        goal.trajectory.points.append(point)

        self.gripper_client.wait_for_server()
        self.get_logger().info('Sending gripper goal...')
        return self.gripper_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = OpenManipulatorController()

    # Example arm pose (radians)
    arm_pose = [0.0, -0.5, 0.3, 0.2]
    node.move_arm(arm_pose, time_sec=3.0)

    rclpy.spin_once(node, timeout_sec=4.0)

    # Close gripper
    node.move_gripper(0.01, time_sec=1.0)
    rclpy.spin_once(node, timeout_sec=2.0)

    # Open gripper
    node.move_gripper(0.0, time_sec=1.0)
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
