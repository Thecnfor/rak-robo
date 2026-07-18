#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

from grasp_demo_interfaces.action import PlanToPose
from grasp_demo_pkg.common import pose_from_point_and_axes


class PlanToPoseNode(Node):
    def __init__(self):
        super().__init__('plan_to_pose_node')
        self.declare_parameter('base_frame', 'base_link_arm')
        self.declare_parameter('workspace_min', [-0.80, -0.70, 0.02])
        self.declare_parameter('workspace_max', [0.80, 0.70, 1.20])
        self.declare_parameter('commanded_pose_topic', '/demo_grasp/commanded_pose')

        # Teaching-only approximate arm motion. This is not IK or MoveIt planning.
        self.declare_parameter('command_topic', '/hand_command')
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('reference_pose', [0.0, 0.58, -1.67, -0.5, 1.51, 0.0])
        self.declare_parameter('execute_steps', 60)
        self.declare_parameter('execute_rate_hz', 20.0)
        self.declare_parameter('hold_seconds', 0.5)

        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter('commanded_pose_topic').value, 10)
        self.joint_pub = self.create_publisher(JointState, self.get_parameter('command_topic').value, 10)
        self.current_demo_pose = [float(v) for v in self.get_parameter('reference_pose').value]

        self.server = ActionServer(
            self,
            PlanToPose,
            '/demo_plan_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            'plan_to_pose_node started; teaching node generates candidate PoseStamped and can execute an approximate fixed-joint motion demo'
        )

    def _in_workspace(self, xyz):
        lo = np.array(self.get_parameter('workspace_min').value, dtype=np.float64)
        hi = np.array(self.get_parameter('workspace_max').value, dtype=np.float64)
        return bool(np.all(xyz >= lo) and np.all(xyz <= hi))

    @staticmethod
    def _clip(value, lo, hi):
        return max(lo, min(hi, float(value)))

    def _target_to_demo_joints(self, xyz, long_axis):
        # This intentionally simple mapping gives students a visible arm motion without
        # solving the real inverse-kinematics problem used in the competition task.
        x, y, z = [float(v) for v in xyz]
        ref = [float(v) for v in self.get_parameter('reference_pose').value]
        if len(ref) != 6:
            ref = [0.0, 0.58, -1.67, -0.5, 1.51, 0.0]

        # Bobac's table target is usually in negative X of base_link_arm. Use that as
        # the forward direction for a coarse base yaw.
        yaw = math.atan2(y, -x if abs(x) > 1e-6 else 1e-6)
        yaw = self._clip(yaw, -1.1, 1.1)

        reach = self._clip(abs(x), 0.25, 0.75)
        height = self._clip(z, 0.05, 0.45)
        reach_t = (reach - 0.25) / 0.50
        height_t = (height - 0.05) / 0.40

        # Conservative shoulder/elbow/wrist values around the observation pose. These
        # are only for a visible demonstration, not precise end-effector placement.
        j2 = 0.55 - 0.30 * reach_t + 0.15 * height_t
        j3 = -1.65 + 0.45 * reach_t - 0.25 * height_t
        j4 = -0.50 + 0.25 * reach_t - 0.10 * height_t
        j5 = 1.45

        # Give joint_6 a small visual alignment cue from the detected long axis.
        axis_yaw = math.atan2(float(long_axis[1]), float(long_axis[0])) if np.linalg.norm(long_axis[:2]) > 1e-6 else 0.0
        j6 = self._clip(axis_yaw, -0.8, 0.8)

        return [yaw, j2, j3, j4, j5, j6]

    def _publish_joint_pose(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(v) for v in positions]
        self.joint_pub.publish(msg)

    def _execute_demo_motion(self, goal_handle, target_pose):
        names = list(self.get_parameter('joint_names').value)
        if len(names) != len(target_pose):
            raise RuntimeError('joint_names length must match generated target pose length')

        start = list(self.current_demo_pose)
        steps = max(2, int(self.get_parameter('execute_steps').value))
        rate_hz = max(1.0, float(self.get_parameter('execute_rate_hz').value))
        period = 1.0 / rate_hz

        feedback = PlanToPose.Feedback()
        feedback.state = 'EXECUTING_APPROXIMATE_JOINT_DEMO'
        goal_handle.publish_feedback(feedback)

        for i in range(steps + 1):
            if goal_handle.is_cancel_requested:
                return False
            t = i / steps
            s = t * t * (3.0 - 2.0 * t)
            pose = [a + (b - a) * s for a, b in zip(start, target_pose)]
            self._publish_joint_pose(names, pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        hold = max(0.0, float(self.get_parameter('hold_seconds').value))
        end = time.time() + hold
        while rclpy.ok() and time.time() < end:
            if goal_handle.is_cancel_requested:
                return False
            self._publish_joint_pose(names, target_pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.current_demo_pose = list(target_pose)
        return True

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = PlanToPose.Result()
        feedback = PlanToPose.Feedback()

        feedback.state = 'CHECKING_WORKSPACE'
        goal_handle.publish_feedback(feedback)
        xyz = np.array([goal.target.point.x, goal.target.point.y, goal.target.point.z], dtype=np.float64)
        lo = np.array(self.get_parameter('workspace_min').value, dtype=np.float64)
        hi = np.array(self.get_parameter('workspace_max').value, dtype=np.float64)
        self.get_logger().info(
            f'{goal.label}: target={xyz.tolist()}, workspace_min={lo.tolist()}, workspace_max={hi.tolist()}, execute={goal.execute}'
        )
        if not self._in_workspace(xyz):
            result.success = False
            result.message = f'target outside workspace: {xyz.tolist()}'
            goal_handle.abort()
            return result

        feedback.state = 'BUILDING_CANDIDATE_POSE'
        goal_handle.publish_feedback(feedback)
        normal = np.array([goal.normal.vector.x, goal.normal.vector.y, goal.normal.vector.z], dtype=np.float64)
        axis = np.array([goal.long_axis.vector.x, goal.long_axis.vector.y, goal.long_axis.vector.z], dtype=np.float64)
        frame_id = goal.target.header.frame_id or self.get_parameter('base_frame').value
        result.commanded_pose = pose_from_point_and_axes(frame_id, self.get_clock().now().to_msg(), xyz, normal, axis)
        self.pose_pub.publish(result.commanded_pose)

        demo_joints = self._target_to_demo_joints(xyz, axis)
        self.get_logger().info(f'{goal.label}: approximate_demo_joints={[round(v, 3) for v in demo_joints]}')

        if goal.execute:
            ok = self._execute_demo_motion(goal_handle, demo_joints)
            if not ok:
                result.success = False
                result.message = 'approximate joint demo canceled'
                goal_handle.canceled()
                return result
            result.message = 'candidate pose published and approximate joint demo executed; no IK/MoveIt trajectory is used'
        else:
            feedback.state = 'POSE_READY'
            goal_handle.publish_feedback(feedback)
            result.message = 'candidate pose generated; set execute=true to run the approximate joint demo'

        result.success = True
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PlanToPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
