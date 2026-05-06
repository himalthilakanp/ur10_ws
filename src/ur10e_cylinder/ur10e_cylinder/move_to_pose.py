#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    CollisionObject,
    PlanningScene,
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class MoveWithMoveIt(Node):

    def __init__(self):
        super().__init__("move_with_moveit")

        # MoveIt action client
        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()

        self.get_logger().info("MoveIt ready ✔")

        # Add obstacle
        self.add_cylinder()

    # -------------------------------------------------
    # ADD CYLINDER OBSTACLE
    # -------------------------------------------------
    def add_cylinder(self):

        pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        scene = PlanningScene()
        scene.is_diff = True

        collision = CollisionObject()
        collision.id = "obstacle"
        collision.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER

        # dimensions = [height, radius]
        primitive.dimensions = [1.0, 0.08]

        pose = PoseStamped()
        pose.header.frame_id = "world"

        pose.pose.position.x = 0.45
        pose.pose.position.y = -0.3
        pose.pose.position.z = 0.5

        pose.pose.orientation.w = 1.0

        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose.pose)

        collision.operation = CollisionObject.ADD

        scene.world.collision_objects.append(collision)

        pub.publish(scene)

        self.get_logger().info("Cylinder added ✔")

        time.sleep(2)

    # -------------------------------------------------
    # MOVE TO JOINT POSITION
    # -------------------------------------------------
    def move_to_joints(self, joints, name="POINT"):

        goal = MoveGroup.Goal()

        # Move group name
        goal.request.group_name = "arm"

        # -------------------------------------------------
        # USE PILZ INDUSTRIAL PLANNER
        # -------------------------------------------------
        goal.request.pipeline_id = "pilz_industrial_motion_planner"

        # Industrial point-to-point motion
        goal.request.planner_id = "PTP"

        # Velocity / acceleration scaling
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Planning settings
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 1

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

            jc.position = float(joints[i])

            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01

            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Planning {name} using PILZ PTP...")

        # -------------------------------------------------
        # SEND GOAL
        # -------------------------------------------------
        send_future = self.client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{name} rejected ❌")
            return

        self.get_logger().info(f"{name} accepted ✔")

        # -------------------------------------------------
        # WAIT FOR RESULT
        # -------------------------------------------------
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is not None:
            self.get_logger().info(f"{name} completed ✔")
        else:
            self.get_logger().error(f"{name} failed ❌")

        time.sleep(1)

    # -------------------------------------------------
    # RUN MOTION SEQUENCE
    # -------------------------------------------------
    def run(self):

        # Original points
        p1 = [1.895, -0.173, 1.452, 0.283, -2.798, 0.0]

        p2 = [0.600, -0.283, 1.825, 0.083, 0.516, 0.0]

        p3 = [0.600, 0.717, -0.170, 0.970, -0.588, 0.0]

        # SAFE waypoint to avoid collision
        safe = [1.2, -1.2, 1.8, 0.0, -1.5, 0.0]

        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Industrial-style deterministic sequence
        sequence = [
            ("P2", p2),

            # go around obstacle
            ("SAFE", safe),

            ("P1", p1),

            ("SAFE", safe),

            ("P2", p2),

            ("P3", p3),

            ("HOME", home),
        ]

        for name, joints in sequence:
            self.move_to_joints(joints, name)

        self.get_logger().info("ALL MOTIONS DONE ✔")


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main(args=None):

    rclpy.init(args=args)

    node = MoveWithMoveIt()

    node.run()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()