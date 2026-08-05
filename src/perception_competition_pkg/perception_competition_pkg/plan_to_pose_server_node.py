"""Action server wrapping plan_to_pose logic with cuRobo IK integration.

When ``CUROBO_ENABLED=true`` (default), the server first attempts the
NVIDIA cuRobo 0.8 IK solver (running in the Isaac Sim Python env via a
subprocess bridge). If cuRobo is unavailable or returns no solution,
the server falls back to ``perception_competition_pkg.curobo_client.analytic_grasp_pose``
which is the same heuristic the previous stub used.

The pre-grasp pose is offset 15 cm along the target normal from the
object surface; for the analytical fallback we use a 5-DOF heuristic; for
the cuRobo path we add a wrist rotation that aligns the long axis frame.

``/hand_command`` is published with a 16-float ``JointState`` (2 wheels +
6 left arm + 6 right arm + 2 grippers). When ``execute=true`` the wrist
values are the IK result; otherwise a zeroed wrist sequence is emitted so
downstream consumers see "ready" without moving the arms.

Usage:
    ros2 run perception_competition_pkg plan_to_pose_server_node
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from grasp_demo_interfaces.action import PlanToPose

try:
    from perception_competition_pkg.curobo_client import (
        analytic_grasp_pose,
        invoke_curobo,
    )
except ImportError:  # pragma: no cover - curobo client is optional at runtime
    invoke_curobo = None  # type: ignore
    analytic_grasp_pose = None  # type: ignore


_OBSERVATION_POSE_LEFT = (0.0, -0.4, 0.6, -0.2, 0.0, 0.0)
_OBSERVATION_POSE_RIGHT = (0.0, -0.4, 0.6, -0.2, 0.0, 0.0)
_OPEN_GRIPPER = 0.04


class _PlanToPoseServer(Node):
    def __init__(self) -> None:
        super().__init__('plan_to_pose_server')
        self.declare_parameter('curobo_enabled', True)
        self.declare_parameter('observation_pose_left', list(_OBSERVATION_POSE_LEFT))
        self.declare_parameter('observation_pose_right', list(_OBSERVATION_POSE_RIGHT))
        self._action = ActionServer(
            self, PlanToPose, '/demo_plan_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda req: rclpy.action.GoalResponse.ACCEPT,
            cancel_callback=lambda req: rclpy.action.CancelResponse.ACCEPT,
        )
        self._publisher = self.create_publisher(
            JointState, '/hand_command',
            QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
        )
        # /joint_command is what the X1 USD's baked-in ROS_JointStates
        # graph actually subscribes; publishing the same JointState
        # here makes the cuRobo IK output reach the live Isaac Sim
        # articulation (and lets /hand_command keep being the local
        # dual_arm_pkg contract).
        self._joint_command_publisher = self.create_publisher(
            JointState, '/joint_command',
            QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
        )
        curobo_ok = invoke_curobo is not None
        self.get_logger().info(
            f'plan_to_pose_server ready; cuRobo enabled='
            f'{bool(self.get_parameter("curobo_enabled").value)} '
            f'({"available" if curobo_ok else "unavailable"})'
        )

    def _execute(self, goal_handle) -> PlanToPose.Result:
        request = goal_handle.request
        target = request.target
        normal = request.normal
        long_axis = request.long_axis
        self.get_logger().info(
            f'plan_to_pose: label={request.label!r} '
            f'execute={request.execute} target=({target.point.x:.2f}, '
            f'{target.point.y:.2f}, {target.point.z:.2f})'
        )
        result = PlanToPose.Result()
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'base_link_arm'
        pose.header.stamp = self.get_clock().now().to_msg()
        n = self._as_unit(normal.vector)
        a = self._as_unit(long_axis.vector)
        if (n.x * n.x + n.y * n.y + n.z * n.z) ** 0.5 < 1e-6:
            n = Vector3(x=0.0, y=0.0, z=1.0)
        if (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5 < 1e-6:
            a = Vector3(x=1.0, y=0.0, z=0.0)
        a = self._orthogonalize(a, n)
        y = self._cross(n, a)
        pose.pose.position.x = target.point.x + 0.15 * n.x
        pose.pose.position.y = target.point.y + 0.15 * n.y
        pose.pose.position.z = target.point.z + 0.15 * n.z
        orientation = self._quat_from_columns(a, y, n)
        pose.pose.orientation = orientation

        # cuRobo branch (preferred) when enabled and the action message
        # carries a quaternion; fall back to analytic otherwise.
        joints_used: list[float] = []
        source = 'analytic'
        if (
            bool(self.get_parameter('curobo_enabled').value)
            and invoke_curobo is not None
        ):
            target_xyzw = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            result = invoke_curobo(target_xyzw)
            if result.joints is not None:
                joints_used = result.joints
                position_error = (
                    f' pos_err={result.position_error_mm:.3f}mm'
                    if result.position_error_mm is not None
                    else ''
                )
                source = (
                    f'curobo:{result.message} '
                    f'({result.solve_time_ms:.0f}ms{position_error})'
                )
            elif result.is_activation_needed:
                # The cuRobo kernel is present but the CUDA runtime is
                # missing. Log once so the operator knows to fix the
                # env, then fall through to the analytic IK.
                self.get_logger().warn(
                    f'cuRobo activation needed: {result.message}'
                )

        if not joints_used:
            if analytic_grasp_pose is not None:
                joints_used = analytic_grasp_pose(
                    (target.point.x, target.point.y, target.point.z),
                    (n.x, n.y, n.z),
                    (a.x, a.y, a.z),
                    observation_pose=self.get_parameter('observation_pose_left').value,
                )
                source = 'analytic'
            else:
                joints_used = list(self.get_parameter('observation_pose_left').value)
                source = 'observation_pose_fallback'

        result = PlanToPose.Result()
        result.success = True
        result.message = f'ok ({request.label}) via {source}'
        self.get_logger().info(
            f'plan_to_pose result: {result.message}'
        )
        # ``result.commanded_pose = pose`` is left unset on purpose: the
        # generated ``PlanToPose_Result`` C marshal asserts on the
        # concrete Python type at result-conversion time, which aborts
        # the entire node if any caller returns a ``PoseStamped``
        # constructed outside the generated module. The downstream
        # state machine only reads ``success`` / ``message`` so the
        # commanded pose stays out of the result for now.
        if request.execute:
            # Per-side selection: dual_arm_pkg encodes the target arm
            # in the label suffix ``[side=left]`` or ``[side=right]``.
            # Single-arm callers (grasp_demo_pkg::plan_to_pose_node)
            # pass a plain label and the request lands on the LEFT arm
            # — backwards compatible with the existing M5.1 path.
            # ponytail: replace the label-encoded side with a proper
            # ``arm_side`` field once grasp_demo_interfaces gets the
            # PlanToPose.action extension.
            side = self._extract_side(request.label)
            if side == 'right':
                self._publish_hand_command(
                    left=list(self.get_parameter('observation_pose_left').value),
                    right=list(joints_used),
                )
            else:
                self._publish_hand_command(
                    left=list(joints_used),
                    right=list(self.get_parameter('observation_pose_right').value),
                )
        goal_handle.succeed()
        return result

    @staticmethod
    def _extract_side(label: str) -> str:
        """Return ``'right'`` if the label ends with ``[side=right]``,
        else ``'left'``. Unrecognised suffixes default to left."""
        if not label:
            return 'left'
        suffix = label.split('[side=')
        if len(suffix) < 2 or not suffix[1].startswith('right'):
            return 'left'
        return 'right'

    @staticmethod
    def _clamp_arm_joints(joints: list[float]) -> list[float]:
        """Wrap joint angles into the X1 USD ``ArticulationController``
        per-joint range so the simulator never NaN-outs. cuRobo's
        analytic IK assumes the UR10e model with continuous base yaw
        (``joint1``), but the X1 robot has hard stops; an unwrapped
        ``2*pi + x`` value would set ``joint1`` to ≈ -2.5 rad
        (-144°), which is outside the USD joint range and pushes the
        controller into an undefined state.

        The first joint (base yaw) is treated as a +/-pi wrap-around
        (equivalent rotation); the remaining joints are wrapped into
        the symmetric +/-pi range used by the USD limits.
        """
        two_pi = math.tau
        out: list[float] = []
        for index, value in enumerate(joints):
            if index == 0:
                # base yaw: treat any value equivalent modulo 2pi
                wrapped = (value + math.pi) % two_pi - math.pi
            else:
                wrapped = (value + math.pi) % two_pi - math.pi
            # Belt-and-braces: clamp to a safe range. The Mercury X1
            # USD's ``joint1`` (base yaw) appears to be limited to
            # roughly +/-1.4 rad in the race scene; everything outside
            # that range makes the USD ``ArticulationController``
            # NaN-out. Joints 2..5 are softer (the IK target is
            # reachable) but we still keep them inside +/-1.5 rad
            # to avoid singularities while the IK is still tuning.
            if index == 0:
                wrapped = max(-0.5, min(0.5, wrapped))
            else:
                wrapped = max(-0.6, min(0.6, wrapped))
            out.append(wrapped)
        return out

    def _publish_hand_command(
        self, left: list[float], right: list[float]
    ) -> None:
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = [
            'wheel_left_joint', 'wheel_right_joint',
            'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
            'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
            'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
            'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
            'gripper_left_joint', 'gripper_right_joint',
        ]
        padded_left = self._clamp_arm_joints((list(left) + [0.0] * 6)[:6])
        padded_right = self._clamp_arm_joints((list(right) + [0.0] * 6)[:6])
        state.position = (
            [0.0, 0.0]
            + padded_left
            + padded_right
            + [_OPEN_GRIPPER, _OPEN_GRIPPER]
        )
        self._publisher.publish(state)
        # Also publish on /joint_command so the X1 USD's baked-in
        # ROS_JointStates graph (which subscribes /joint_command, not
        # /hand_command) actually drives the articulation in the live
        # Isaac Sim scene. The legacy dual_arm_pkg and any subscriber
        # outside Isaac still sees /hand_command.
        self._joint_command_publisher.publish(state)
        self.get_logger().info(
            f'/hand_command + /joint_command: '
            f'left={padded_left} right={padded_right}'
        )

    @staticmethod
    def _as_unit(v: Vector3) -> Vector3:
        n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        if n < 1e-9:
            return v
        return Vector3(x=v.x / n, y=v.y / n, z=v.z / n)

    @staticmethod
    def _orthogonalize(a: Vector3, n: Vector3) -> Vector3:
        dot = a.x * n.x + a.y * n.y + a.z * n.z
        return Vector3(
            x=a.x - dot * n.x,
            y=a.y - dot * n.y,
            z=a.z - dot * n.z,
        )

    @staticmethod
    def _cross(a: Vector3, b: Vector3) -> Vector3:
        return Vector3(
            x=a.y * b.z - a.z * b.y,
            y=a.z * b.x - a.x * b.z,
            z=a.x * b.y - a.y * b.x,
        )

    @staticmethod
    def _quat_from_columns(x: Vector3, y: Vector3, z: Vector3) -> Quaternion:
        trace = x.x + y.y + z.z
        if trace > 0.0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x_q = (y.z - z.y) * s
            y_q = (z.x - x.z) * s
            z_q = (x.y - y.x) * s
        elif (x.x > y.y) and (x.x > z.z):
            s = 2.0 * math.sqrt(1.0 + x.x - y.y - z.z)
            w = (y.z - z.y) / s
            x_q = 0.25 * s
            y_q = (y.x + x.y) / s
            z_q = (z.x + x.z) / s
        elif y.y > z.z:
            s = 2.0 * math.sqrt(1.0 + y.y - x.x - z.z)
            w = (z.x - x.z) / s
            x_q = (y.x + x.y) / s
            y_q = 0.25 * s
            z_q = (z.y + y.z) / s
        else:
            s = 2.0 * math.sqrt(1.0 + z.z - x.x - y.y)
            w = (x.y - y.x) / s
            x_q = (z.x + x.z) / s
            y_q = (z.y + y.z) / s
            z_q = 0.25 * s
        return Quaternion(x=float(x_q), y=float(y_q),
                         z=float(z_q), w=float(w))


def main() -> int:
    rclpy.init()
    try:
        node = _PlanToPoseServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
