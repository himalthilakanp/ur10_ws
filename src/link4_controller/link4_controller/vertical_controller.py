#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class VerticalController(Node):

    def __init__(self):
        super().__init__('vertical_controller')

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)

        self.get_logger().info("Link4 Vertical Controller Started")

    def callback(self, msg):

        names = msg.name
        pos = list(msg.position)

        try:
            i2 = names.index('shoulder_lift_joint')
            i3 = names.index('elbow_joint')
            i4 = names.index('wrist_1_joint')
        except:
            return

        q2 = pos[i2]
        q3 = pos[i3]

        # Keep link4 vertical
        q4 = math.pi - (q2 + q3)

        pos[i4] = q4

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = pos

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VerticalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()