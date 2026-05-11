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
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import ObjectColor
from std_msgs.msg import ColorRGBA


class MoveWithMoveIt(Node):

    def __init__(self):
        super().__init__("move_with_moveit")

        # MoveIt action client
        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()

        self.get_logger().info("MoveIt ready ✔")

        # Planning scene publisher
        self.scene_pub = self.create_publisher(
        PlanningScene,
        '/planning_scene',
        10
        )

        # Add obstacle
        self.add_cylinder()

        self.add_leaves()

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

        pub.publish(scene)

        self.get_logger().info("Cylinder added ✔")

        time.sleep(2)

    # =================================================
    # ADD LEAVES
    # =================================================
    def add_leaves(self):

        GOLDEN_ANGLE = math.radians(137.5)

        CYLINDER_X = 0.45
        CYLINDER_Y = -0.3

        STEM_RADIUS = 0.015

        NUM_LEAVES = 24

        TOTAL_HEIGHT = 1.35

        LEAF_RADIUS = STEM_RADIUS + 0.04

        # -------------------------------------------------
        # FIRST LEAF OPPOSITE DIRECTION
        # -------------------------------------------------

        START_ANGLE = math.pi

        for i in range(NUM_LEAVES):

            scene = PlanningScene()

            scene.is_diff = True

            leaf = CollisionObject()

            leaf.id = f"leaf_{i}"

            leaf.header.frame_id = "world"

            # -------------------------------------------------
            # LEAF SHAPE
            # -------------------------------------------------

            primitive = SolidPrimitive()

            primitive.type = SolidPrimitive.BOX

            # [length, width, thickness]
            primitive.dimensions = [0.06, 0.015, 0.005]

            # -------------------------------------------------
            # GOLDEN ANGLE ARRANGEMENT
            # -------------------------------------------------

            theta = START_ANGLE + i * GOLDEN_ANGLE

            # first leaf at 20 cm height
            z = 0.20 + (i / NUM_LEAVES) * TOTAL_HEIGHT

            x = CYLINDER_X + LEAF_RADIUS * math.cos(theta)

            y = CYLINDER_Y + LEAF_RADIUS * math.sin(theta)

            pose = PoseStamped()

            pose.header.frame_id = "world"

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # -------------------------------------------------
            # LEAF POINTS TOWARD STEM CENTER
            # -------------------------------------------------

            yaw = theta + math.pi

            q = self.euler_to_quaternion(
                0.0,
                0.0,
                yaw
            )

            pose.pose.orientation = q

            leaf.primitives.append(primitive)

            leaf.primitive_poses.append(pose.pose)

            leaf.operation = CollisionObject.ADD

            scene.world.collision_objects.append(leaf)

            # -------------------------------------------------
            # ORANGE LEAF COLOR
            # -------------------------------------------------

            color = ObjectColor()

            color.id = leaf.id

            color.color = ColorRGBA(
                r=1.0,
                g=0.45,
                b=0.0,
                a=1.0
            )

            scene.object_colors.append(color)

            self.scene_pub.publish(scene)

            time.sleep(0.03)

        self.get_logger().info(
            "24 golden-angle leaves added ✔"
        )

        time.sleep(2)

    # =================================================
    # EULER TO QUATERNION
    # =================================================
    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = (
            math.sin(roll / 2) *
            math.cos(pitch / 2) *
            math.cos(yaw / 2)
            -
            math.cos(roll / 2) *
            math.sin(pitch / 2) *
            math.sin(yaw / 2)
        )

        qy = (
            math.cos(roll / 2) *
            math.sin(pitch / 2) *
            math.cos(yaw / 2)
            +
            math.sin(roll / 2) *
            math.cos(pitch / 2) *
            math.sin(yaw / 2)
        )

        qz = (
            math.cos(roll / 2) *
            math.cos(pitch / 2) *
            math.sin(yaw / 2)
            -
            math.sin(roll / 2) *
            math.sin(pitch / 2) *
            math.cos(yaw / 2)
        )

        qw = (
            math.cos(roll / 2) *
            math.cos(pitch / 2) *
            math.cos(yaw / 2)
            +
            math.sin(roll / 2) *
            math.sin(pitch / 2) *
            math.sin(yaw / 2)
        )

        q = Quaternion()

        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw

        return q


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
    # -------------------------------------------------1
    def run(self):

        # Original points
        p1 = [1.944, -0.171, 1.566, 0.154, -2.908, 0.0]

        p2 = [0.572, -0.222, 1.776, 0.026, 0.536, 0.0]

        p3 = [0.629, 0.431, 0.162, 0.939, -0.716, 0.0]

        #---------------------------------------------------2

        p4 = [0.674, -0.045, 0.641, 0.977, -0.814, 0.0]

        p5 = [0.624, -1.015, 2.384, 0.186, 0.633, 0.0]

        p6 = [1.919, -0.834, 2.018, 0.401, -2.856, 0.0]

        #----------------------------------------------------3

        p7 = [1.910, -1.508, 2.293, 0.788, -2.875, 0.0]

        p8 = [0.534, -1.637, 2.452, 0.762, 0.388, 0.0]

        p9 = [0.616, -0.305, 0.653, 1.188, -0.868, 0.0]

        #----------------------------------------------------4
        
        p10 = [0.667, -0.908, 1.157, 1.320, -0.705, 0.0]

        p11 = [0.571, -2.383, 2.567, 1.395, 0.516, 0.0]

        p12 = [2.071, -1.942, 2.203, 1.313, -2.817, 0.0]

        #-----------------------------------------------------5
        p13 = [1.915, -2.071, 2.011, 1.621, -2.862, 0.0]

        p14 = [0.651, -2.783, 2.463, 1.871, 0.736, 0.0]

        p15 = [0.680, -0.502, 0.261, 1.795, -0.898, 0.0]

        #------------------------------------------------------6
        p16 = [0.668, -0.034, -0.215, -1.324, 0.917, 0.0]

        p17 = [0.591, -2.602, 2.629, -1.652, -0.656, 0.0]

        p18 = [1.901, -2.003, 2.263, -1.829, 2.859, 0.0]

        #-------------------------------------------------------7
        p19 = [1.918, -2.055, 1.902, -1.448, 2.924, 0.0]

        p20 = [0.606, -2.501, 2.244, -1.339, -0.600, 0.0]

        p21 = [0.652, -0.631, 0.328, -1.320, 0.804, 0.0]

        #--------------------------------------------------------8

        p22 = [0.683, -0.695, 0.053, -0.958, 0.687, 0.0] 
        
        p23 = [0.764, 0.116, -1.767, 0.079, -0.821, 0.0] 
        
        p24 = [1.878, -0.046, -1.213, -0.294, 2.769, 0.0]

        # SAFE waypoint to avoid collision
        safe = [1.2, -1.2, 1.8, 0.0, -1.5, 0.0]

        # Slight motion from P2 toward leaf
        leaf_pose = [0.620, -0.180, 1.720, 0.026, 0.536, 0.0]

        p11_p12_safe = [1.25, -1.65, 2.15, 0.35, -1.20, 0.0]

        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Industrial-style deterministic sequence
        sequence = [
            ("P2", p2),

            ("LEAF", leaf_pose),

            #("P1", p1),

            #("P2", p2),

            #("P3", p3),

            #("P4", p4),

            #("P5", p5),

            #("P6", p6),

            #("P7", p7),

            #("P8", p8),

            #("P9", p9),

            #("P10", p10),

            #("P11", p11),

            #("P12", p12),

            #("P13", p13),

            #("P14", p14),

            #("P15", p15),

            #("P16", p16),

            #("P17", p17),

            #("P18", p18),

            #("P19", p19),

            #("P20", p20),

            #("P21", p21),

            #("P22", p22),

            #("P23", p23),

            #("P24", p24),

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