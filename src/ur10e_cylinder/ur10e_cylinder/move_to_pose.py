#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MovePoints(Node):

    def __init__(self):
        super().__init__("move_points")

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.get_logger().info("Waiting for controller server...")
        self.client.wait_for_server()
        self.get_logger().info("Connected")

    def move_to_joints(self, joint_values, name="POINT", duration_sec=4):

        goal = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_values          # exact positions, no tolerance
        point.velocities = [0.0] * 6           # stop at the point
        point.time_from_start = Duration(sec=duration_sec)

        traj.points.append(point)
        goal.trajectory = traj

        # Tight goal tolerance — controller enforces this, not OMPL
        from control_msgs.msg import JointTolerance
        for jname in self.joint_names:
            tol = JointTolerance()
            tol.name = jname
            tol.position = 0.001               # 0.001 rad = exact
            goal.goal_tolerance.append(tol)

        goal.goal_time_tolerance = Duration(sec=2)

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
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{name} reached ✔")
            return True
        else:
            self.get_logger().error(f"{name} failed ❌ code {result.error_code}")
            return False

    def run(self):

        # ================= POINT 1 =================
        p1 = [
            ("shoulder_pan_joint",   0.245),
            ("shoulder_lift_joint",  -0.053),
            ("elbow_joint",          1.780),
            ("wrist_1_joint",       -0.158),
            ("wrist_2_joint",        -0.320),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 2 =================
        p2 = [
            ("shoulder_pan_joint",   0.117),
            ("shoulder_lift_joint",  0.030),
            ("elbow_joint",          2.036),
            ("wrist_1_joint",       -0.505),
            ("wrist_2_joint",        0.000),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 3 =================
        p3 = [
            ("shoulder_pan_joint",   1.350),
            ("shoulder_lift_joint", -0.094),
            ("elbow_joint",          1.908),
            ("wrist_1_joint",       -0.237),
            ("wrist_2_joint",       -3.239),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 4 =================
        '''p4 = [
            ("shoulder_pan_joint",   0.264),
            ("shoulder_lift_joint", -0.472),
            ("elbow_joint",          1.744),
            ("wrist_1_joint",        0.088),
            ("wrist_2_joint",       -0.474),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 5 =================
        p5 = [
            ("shoulder_pan_joint",   0.000),
            ("shoulder_lift_joint", -0.443),
            ("elbow_joint",          2.224),
            ("wrist_1_joint",       -0.215),
            ("wrist_2_joint",       -0.275),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 6 =================
        p6 = [
            ("shoulder_pan_joint",   1.863),
            ("shoulder_lift_joint", -0.407),
            ("elbow_joint",          2.067),
            ("wrist_1_joint",       -0.136),
            ("wrist_2_joint",       -2.802),
            ("wrist_3_joint",        0.000),
        ]
        # ================= POINT 7 =================
        p7 = [
            ("shoulder_pan_joint",   0.305),
            ("shoulder_lift_joint", -0.530),
            ("elbow_joint",          1.360),
            ("wrist_1_joint",        0.737),
            ("wrist_2_joint",       -0.865),
            ("wrist_3_joint",        0.000),
        ]'''
        # ================= HOME =================
        home = [
            ("shoulder_pan_joint",   0.0),
            ("shoulder_lift_joint",  0.0),
            ("elbow_joint",          0.0),
            ("wrist_1_joint",        0.0),
            ("wrist_2_joint",        0.0),
            ("wrist_3_joint",        0.0),
        ]
        # ================= SEQUENCE =================
        sequence = [
            ("P2",   p2),
            ("P3",   p3),
            ("P2",   p2),
            ("P1",   p1),
            ("HOME", home),
        ]

        for name, joints in sequence:
            ok = self.move_to_joints([v for _, v in joints], name, duration_sec=4)
            if not ok:
                self.get_logger().error("Sequence stopped.")
                return
            time.sleep(1)

        self.get_logger().info("DONE ✔")


def main(args=None):
    rclpy.init(args=args)
    node = MovePoints()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()