#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


class InitialPose(Node):
    def __init__(self):
        super().__init__('init_pos')

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Wait for subscribers to connect
        self.create_timer(3.0, self.publish_initial_pose)

        self.get_logger().info('Initial pose node started, publishing in 3 seconds...')
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return

        checkpoint = PoseWithCovarianceStamped()
        checkpoint.header.frame_id = 'map'
        checkpoint.header.stamp = self.get_clock().now().to_msg()

        # Use $ ros2 topic echo /odom --once to obtain poses (both position and orientation)

        checkpoint.pose.pose.position.x = -1.99968900545
        checkpoint.pose.pose.position.y = -0.499122759107
        checkpoint.pose.pose.position.z = -0.0010073990697

        [x, y, z, w] = quaternion_from_euler(0.0, 0.0, 0.0)
        checkpoint.pose.pose.orientation.x = -6.03836998093e-06
        checkpoint.pose.pose.orientation.y = 0.00158963960308
        checkpoint.pose.pose.orientation.z = 0.00275331003065
        checkpoint.pose.pose.orientation.w = 0.999994946134

        self.get_logger().info(f'{checkpoint}')
        self.pub.publish(checkpoint)
        self.published = True
        self.get_logger().info('Initial pose published.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
