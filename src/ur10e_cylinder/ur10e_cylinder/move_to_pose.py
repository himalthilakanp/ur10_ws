#!/usr/bin/env python3

import rclpy
import time
import math

from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    CollisionObject,
    PlanningScene,
    ObjectColor,
    AttachedCollisionObject
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveWithMoveIt(Node):

    def __init__(self):
        super().__init__("move_with_moveit")

        # MoveIt client
        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()
        self.get_logger().info("MoveIt ready ✔")

        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self.get_logger().info("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()
        self.get_logger().info("Gripper ready ✔")

        # Planning scene publisher
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.add_cylinder()
        self.add_leaves()

    # -------------------------------------------------
    # OBSTACLE
    # -------------------------------------------------
    def add_cylinder(self):

        scene = PlanningScene()
        scene.is_diff = True

        collision = CollisionObject()
        collision.id = "obstacle"
        collision.header.frame_id = "world"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [1.6, 0.015]

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

        self.scene_pub.publish(scene)

        self.get_logger().info("Cylinder added ✔")
        time.sleep(2)

    # -------------------------------------------------
    # LEAVES
    # -------------------------------------------------
    def add_leaves(self):

        GOLDEN_ANGLE = math.radians(137.5)

        NUM_LEAVES = 24
        TOTAL_HEIGHT = 1.35
        LEAF_RADIUS = 0.055

        START_ANGLE = math.pi

        for i in range(NUM_LEAVES):

            scene = PlanningScene()
            scene.is_diff = True

            leaf = CollisionObject()
            leaf.id = f"leaf_{i}"
            leaf.header.frame_id = "world"

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.06, 0.015, 0.005]

            theta = START_ANGLE + i * GOLDEN_ANGLE
            z = 0.20 + (i / NUM_LEAVES) * TOTAL_HEIGHT

            x = 0.45 + LEAF_RADIUS * math.cos(theta)
            y = -0.3 + LEAF_RADIUS * math.sin(theta)

            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            yaw = theta + math.pi
            q = self.euler_to_quaternion(0, 0, yaw)
            pose.pose.orientation = q

            leaf.primitives.append(primitive)
            leaf.primitive_poses.append(pose.pose)
            leaf.operation = CollisionObject.ADD

            scene.world.collision_objects.append(leaf)

            color = ObjectColor()
            color.id = leaf.id
            color.color = ColorRGBA(r=1.0, g=0.45, b=0.0, a=1.0)

            scene.object_colors.append(color)

            self.scene_pub.publish(scene)
            time.sleep(0.03)

        self.get_logger().info("Leaves added ✔")
        time.sleep(2)

    # -------------------------------------------------
    # ATTACH LEAF (KEY PART)
    # -------------------------------------------------
    def attach_leaf(self, leaf_id):

        scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        attached = AttachedCollisionObject()
        attached.object.id = leaf_id
        attached.object.header.frame_id = "world"
        attached.object.operation = CollisionObject.ADD

        attached.link_name = "tool0"  # IMPORTANT: change if needed

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        scene_pub.publish(scene)

        self.get_logger().info(f"Leaf {leaf_id} attached ✔")

    # -------------------------------------------------
    # GRIPPER
    # -------------------------------------------------
    def move_gripper(self, position):

        goal = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = ["left_finger_joint"]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1

        traj.points.append(point)
        goal.trajectory = traj

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper rejected ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Gripper done ✔")
    
    def pluck_motion(self, current_pose):
        """
        Adds real plucking behavior:
        - slight downward tilt
        - half rotation
        - lift
        """

        # unpack current joints
        j = list(current_pose)

        # 1. small downward tilt (wrist_1_joint)
        tilt_pose = j.copy()
        tilt_pose[3] += 0.4   # pitch down

        self.move_to_joints(tilt_pose, "TILT_DOWN")

        # 2. half rotation (wrist_3_joint)
        twist_pose = tilt_pose.copy()
        twist_pose[5] += math.pi / 2   # 90° twist

        self.move_to_joints(twist_pose, "TWIST")

        # 3. slight pull upward
        lift_pose = twist_pose.copy()
        lift_pose[2] += 0.05  # small lift in joint-space approximation

        self.move_to_joints(lift_pose, "LIFT_AFTER_PLUCK")

    # -------------------------------------------------
    # MOVE
    # -------------------------------------------------
    def move_to_joints(self, joints, name):

        goal = MoveGroup.Goal()

        goal.request.group_name = "arm"
        goal.request.pipeline_id = "pilz_industrial_motion_planner"
        goal.request.planner_id = "PTP"

        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        constraints = Constraints()

        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = float(joints[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{name} rejected ❌")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(f"{name} done ✔")

    # -------------------------------------------------
    # RUN PICK PIPELINE
    # -------------------------------------------------
    def run(self):

        p2 = [0.572, -0.222, 1.776, 0.026, 0.536, 0.0]
        leaf_pose = [0.817, -0.230, 1.362, 0.441, 0.275, 0.0]
        home = [0, 0, 0, 0, 0, 0]

        sequence = [
            ("P2", p2),

            ("GRIP_OPEN", None),

            ("APPROACH", leaf_pose),

            ("GRIP_CLOSE", None),

            ("ATTACH", "leaf_1"),

            ("TWIST_TEST", leaf_pose),

            ("LIFT", p2),

            ("HOME", home),
        ]

        for name, data in sequence:

            if name == "GRIP_OPEN":
                self.move_gripper(0.0)
                continue

            if name == "GRIP_CLOSE":
                self.move_gripper(0.035)
                continue

            if name == "ATTACH":
                self.attach_leaf(data)
                continue
            
            if name == "TWIST_TEST":

                test_pose = data.copy()

                # rotate wrist_3_joint
                test_pose[5] += math.pi / 2

                self.move_to_joints(test_pose, "TWIST_TEST")

                continue

            self.move_to_joints(data, name)

        self.get_logger().info("PICK COMPLETE ✔")

    # -------------------------------------------------
    # QUAT
    # -------------------------------------------------
    def euler_to_quaternion(self, roll, pitch, yaw):

        q = Quaternion()

        q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return q


def main(args=None):
    rclpy.init(args=args)
    node = MoveWithMoveIt()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()