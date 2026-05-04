#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class MoveWithMoveIt(Node):

    def __init__(self):
        super().__init__("move_with_moveit")

        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()

        self.get_logger().info("MoveIt ready ✔")

        self.add_cylinder()

    # -------------------------
    # ADD OBSTACLE
    # -------------------------
    def add_cylinder(self):

        pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        scene = PlanningScene()
        scene.is_diff = True

        collision = CollisionObject()
        collision.id = "obstacle"
        collision.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [1.6, 0.15]  # height, radius

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.45
        pose.pose.position.y = -0.3
        pose.pose.position.z = 0.8
        pose.pose.orientation.w = 1.0

        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose.pose)
        collision.operation = CollisionObject.ADD

        scene.world.collision_objects.append(collision)

        pub.publish(scene)

        self.get_logger().info("Cylinder added ✔")
        time.sleep(2)

    # -------------------------
    # MOVE TO JOINTS
    # -------------------------
    def move_to_joints(self, joints, name="POINT"):

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"

        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        constraints = Constraints()

        for i in range(len(joints)):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]

            # 🔥 FIX: force float conversion (this fixes your crash)
            jc.position = float(joints[i])
            jc.tolerance_above = float(0.01)
            jc.tolerance_below = float(0.01)
            jc.weight = float(1.0)

            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Planning {name}...")

        # -------------------------
        # ACTION CALL (SAFE)
        # -------------------------
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{name} rejected ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(f"{name} done ✔")

        # small safety delay to avoid controller overlap
        time.sleep(1)

    # -------------------------
    # SEQUENCE
    # -------------------------
    def run(self):

        p1 = [0.245, -0.053, 1.780, -0.158, -0.320, 0.0]
        p2 = [0.117, 0.030, 2.036, -0.505, 0.0, 0.0]
        p3 = [1.350, -0.094, 1.908, -0.237, -3.239, 0.0]
        home = [0, 0, 0, 0, 0, 0]

        sequence = [
            ("P2", p2),
            ("P3", p3),
            ("P2", p2),
            ("P1", p1),
            ("HOME", home),
        ]

        for name, joints in sequence:
            self.move_to_joints(joints, name)

        self.get_logger().info("DONE ✔")


def main(args=None):
    rclpy.init(args=args)
    node = MoveWithMoveIt()

    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

