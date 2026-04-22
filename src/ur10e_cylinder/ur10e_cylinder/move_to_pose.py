#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class MoveNode(Node):

    def __init__(self):
        super().__init__("move_to_pose_client")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Waiting for MoveGroup server...")
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):

        goal = MoveGroup.Goal()

        goal.request.group_name = "arm"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5

        '''joints = [
            ("shoulder_pan_joint", 0.0),
            ("shoulder_lift_joint", 0.0),
            ("elbow_joint", 0.0),
            ("wrist_1_joint", 0.0),
            ("wrist_2_joint", 0.0),
            ("wrist_3_joint", 1.57)
        ]'''
        joints = [
            ("shoulder_pan_joint", 1.57),      # rotate base 90°
            ("shoulder_lift_joint", 1.0),
            ("elbow_joint", 1.2),
            ("wrist_1_joint", -1.0),
            ("wrist_2_joint", 0.8),
            ("wrist_3_joint", 2.5)
        ]
        '''joints = [
            ("shoulder_pan_joint", 0.0),
            ("shoulder_lift_joint", 0.0),
            ("elbow_joint", 0.0),
            ("wrist_1_joint", 0.0),
            ("wrist_2_joint", 0.0),
            ("wrist_3_joint", 1.57)
        ]'''

        constraints = Constraints()

        for name, value in joints:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info("Sending motion goal...")
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = MoveNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()