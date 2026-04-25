#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener


class EndEffectorPosition(Node):

    def __init__(self):
        super().__init__('ee_position')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.timer = self.create_timer(
            0.5,
            self.show_position
        )

        self.get_logger().info(
            "End Effector Position Node Started"
        )

    def show_position(self):

        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_tip',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            self.get_logger().info(
                f"tool_tip Position -> "
                f"X:{x:.3f}  "
                f"Y:{y:.3f}  "
                f"Z:{z:.3f}"
            )

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()