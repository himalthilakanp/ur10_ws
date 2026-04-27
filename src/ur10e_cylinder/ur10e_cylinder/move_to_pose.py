#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class MovePoints(Node):

    def __init__(self):
        super().__init__("move_points")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Waiting for MoveGroup server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected")

    # --------------------------------------------------
    def move_joint_point(self, joints, name="POINT"):

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"

        goal.request.allowed_planning_time = 15.0
        goal.request.num_planning_attempts = 80
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15

        constraints = Constraints()

        for joint_name, joint_val in joints:

            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_val
            jc.tolerance_above = 0.15
            jc.tolerance_below = 0.15
            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Moving to {name} ...")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error(f"{name} rejected ❌")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info(f"{name} reached ✔")
            return True
        else:
            self.get_logger().error(
                f"{name} failed ❌ code {result.error_code.val}"
            )
            return False

    # --------------------------------------------------
    def run(self):

        # ================= POINT 1 =================
        p1 = [
            ("shoulder_pan_joint",   0.000),
            ("shoulder_lift_joint", -2.087),
            ("elbow_joint",          2.008),
            ("wrist_1_joint",        1.681),
            ("wrist_2_joint",        0.000),
            ("wrist_3_joint",        0.000),
        ]

        # ================= POINT 2 =================
        # ADD YOUR VALUES HERE
        p2 = [
            ("shoulder_pan_joint",   1.816),
            ("shoulder_lift_joint", -1.679),
            ("elbow_joint",          1.623),
            ("wrist_1_joint",        1.579),
            ("wrist_2_joint",       -2.766),
            ("wrist_3_joint",        0.000),
        ]

        # ================= POINT 3 =================
        # ADD YOUR VALUES HERE
        p3 = [
            ("shoulder_pan_joint",   0.357),
            ("shoulder_lift_joint", -1.103),
            ("elbow_joint",          1.070),
            ("wrist_1_joint",        1.647),
            ("wrist_2_joint",        -0.899),
            ("wrist_3_joint",        0.000),
        ]

        # ================= HOME =================
        home = [
            ("shoulder_pan_joint",   0.0),
            ("shoulder_lift_joint",  0.0),
            ("elbow_joint",          0.0),
            ("wrist_1_joint",        0.0),
            ("wrist_2_joint",        0.0),
            ("wrist_3_joint",        0.0),
        ]

        # ===== EXECUTION ORDER =====
        sequence = [
            ("P1", p1),
            ("P2", p2),
            ("P1", p1),
            ("P3", p3),
            ("HOME", home),
        ]

        for name, point in sequence:

            ok = self.move_joint_point(point, name)

            if not ok:
                self.get_logger().error("Sequence stopped.")
                return

            time.sleep(3)

        self.get_logger().info("DONE ✔")


def main(args=None):
    rclpy.init(args=args)

    node = MovePoints()
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

