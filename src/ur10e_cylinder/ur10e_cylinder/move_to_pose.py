#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters,
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MoveItErrorCodes
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
import trajectory_msgs.msg as traj


class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose')
        self._client = ActionClient(self, MoveGroup, '/move_action')

        self._traj_pub = self.create_publisher(
            traj.JointTrajectory,
            '/arm_controller/joint_trajectory',
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE
            )
        )

    def quat_from_rpy(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5);  sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw

    def go_home(self):
        self.get_logger().info('Moving to home position...')
        msg = traj.JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = traj.JointTrajectoryPoint()
        point.positions = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 4
        msg.points = [point]

        time.sleep(1.0)
        self._traj_pub.publish(msg)
        self.get_logger().info('Home command sent, waiting...')
        time.sleep(5.0)

    def send_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.get_logger().info(f'Planning to x={x} y={y} z={z}')
        self._client.wait_for_server()

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()

        req.group_name = 'arm'
        req.start_state.is_diff = True
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 20
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = 'base_link'
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05]

        bv = BoundingVolume()
        bv.primitives = [primitive]
        bv.primitive_poses = [target_pose]

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tool0'
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base_link'
        ori_constraint.link_name = 'tool0'
        ori_constraint.orientation.x = qx
        ori_constraint.orientation.y = qy
        ori_constraint.orientation.z = qz
        ori_constraint.orientation.w = qw
        ori_constraint.absolute_x_axis_tolerance = 0.5
        ori_constraint.absolute_y_axis_tolerance = 0.5
        ori_constraint.absolute_z_axis_tolerance = 0.5
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        req.goal_constraints = [constraints]

        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.planning_scene_diff.is_diff = True

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by move_group!')
            return

        self.get_logger().info('Goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        code = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Motion SUCCESS!')
        else:
            error_map = {
                v: k for k, v in MoveItErrorCodes.__dict__.items()
                if isinstance(v, int)
            }
            self.get_logger().error(
                f'Motion FAILED: {error_map.get(code, "UNKNOWN")} (code={code})'
            )


def main():
    rclpy.init()
    node = MoveToPose()

    # Go home first
    node.go_home()

    # Use EXACT orientation from tf2_echo output
    # Quaternion (xyzw) [-0.707, 0.000, 0.000, 0.707]
    qx = -0.707
    qy =  0.000
    qz =  0.000
    qw =  0.707

    # Target very close to current position (0.735, 0.115, 0.889)
    node.send_goal(
        x=0.70,
        y=0.10,
        z=0.85,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
