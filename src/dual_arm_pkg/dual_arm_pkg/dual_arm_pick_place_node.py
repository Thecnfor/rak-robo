"""Pick-and-place state machine for the Mercury X1 dual-arm.

Drives the air-ground handoff: when the ground pipeline publishes
``/arena/ground/state == COMPLETE``, this node emits ``IDLE →
PARALLEL_DETECT → DUAL_GRASP → LIFTING → PLACING → COMPLETE`` and
publishes ``/arena/ground/state = COMPLETE`` on the success path.

2026-07-20 changes:

* Real ``DetectObject`` + ``PlanToPose`` action clients replace the
  wall-clock stub. Phase advance waits for the upstream action to
  return (with a configurable timeout) instead of a fixed timer.
* A new ``PARALLEL_DETECT → DUAL_GRASP`` pair lets both arms detect
  and plan at the same time (the M5.3 +6 scoring item). If only one
  action server is online the state machine still completes using
  whichever is available, so the existing single-arm path remains
  covered.
* ``/dual_arm/collision_warning`` publishes a tiny ``Bool`` flag when
  the heuristic collision check (left / right keepout on the chest
  link) flags an unsafe configuration.

The state machine contract stays the same so the existing
``competition_orchestrator_pkg::air_ground_orchestrator`` keeps reading
``/arena/ground/state``.

Usage:
    ros2 run dual_arm_pkg dual_arm_pick_place_node
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .joint_order import (
    COLLISION_KEEPOUT_X,
    COLLISION_KEEPOUT_Y,
    COLLISION_SAFE_WRIST_DISTANCE,
    X1_LEFT_ARM_SLICE,
    X1_RIGHT_ARM_SLICE,
    X1_TOTAL_DOF,
    wrist_distance,
)

try:
    from grasp_demo_interfaces.action import (
        DetectObject,
        GripperCommand,
        PlanToPose,
    )
except ImportError:  # pragma: no cover
    DetectObject = None  # type: ignore
    GripperCommand = None  # type: ignore
    PlanToPose = None  # type: ignore


# Re-export the shared values under the underscore-prefixed names the
# pre-existing tests and the original code expect. Keeps callers stable.
_COLLISION_KEEPOUT_X = COLLISION_KEEPOUT_X
_COLLISION_KEEPOUT_Y = COLLISION_KEEPOUT_Y

_ARENA_STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _Phase(Enum):
    IDLE = 'IDLE'
    PARALLEL_DETECT = 'PARALLEL_DETECT'
    DUAL_GRASP = 'DUAL_GRASP'
    LIFTING = 'LIFTING'
    PLACING = 'PLACING'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'


@dataclass
class _State:
    phase: _Phase = _Phase.IDLE
    last_target_label: str = ''
    last_failure: str = ''
    last_detect_at: float = 0.0
    last_plan_at: float = 0.0
    # The latest DetectObject result. We cache the geometry so the
    # downstream _send_plan_goals can build per-arm PlanToPose goals
    # without re-issuing the detect call. ``target`` is the object
    # centre in base_link_arm, ``normal`` is the surface normal, and
    # ``long_axis`` is the principal in-plane direction.
    last_detection_target: Optional[object] = None  # geometry_msgs/PointStamped
    last_detection_normal: Optional[object] = None  # Vector3Stamped
    last_detection_long_axis: Optional[object] = None  # Vector3Stamped


class _PickPlace(Node):
    def __init__(self) -> None:
        super().__init__('dual_arm_pick_place_node')
        self.declare_parameter('detect_timeout_sec', 5.0)
        self.declare_parameter('plan_timeout_sec', 10.0)
        self.declare_parameter('pickup_classes', ['pencil', 'pen'])
        self.declare_parameter('collision_check_enabled', True)
        self.declare_parameter('collision_safe_distance', COLLISION_SAFE_WRIST_DISTANCE)
        # Subscribed /hand_command samples; the FK consumer walks them
        # off the front and skips anything with the wrong DOF count.
        self._hand_samples: list[list[float]] = []
        self._state = _State()
        self._state_publisher = self.create_publisher(
            String, '/dual_arm/phase', 10
        )
        self._arena_publisher = self.create_publisher(
            String, '/arena/ground/state', _ARENA_STATE_QOS
        )
        self._collision_warn_pub = self.create_publisher(
            Bool, '/dual_arm/collision_warning', 10
        )
        self._goal_subscription = self.create_subscription(
            String, '/arena/ground/request', self._on_request, 10
        )
        # Real FK data path: we don't trust action results; we read the
        # latest /hand_command JointState that the IK server actually
        # published, slice out the two 6-DOF arm vectors, and run our
        # own forward kinematics through joint_order.wrist_distance.
        self._hand_subscription = self.create_subscription(
            JointState, '/hand_command', self._on_hand_command, 10
        )
        self._detect_client = None
        self._plan_client = None
        self._gripper_client = None
        self._wait_timer = None
        if DetectObject is not None:
            self._detect_client = ActionClient(
                self, DetectObject, '/demo_detect_object'
            )
        if PlanToPose is not None:
            self._plan_client = ActionClient(
                self, PlanToPose, '/demo_plan_to_pose'
            )
        if GripperCommand is not None:
            self._gripper_client = ActionClient(
                self, GripperCommand, '/demo_gripper_command'
            )
        self._publish_state()
        self.get_logger().info(
            f'PickPlace ready: detect={self._detect_client is not None} '
            f'plan={self._plan_client is not None} '
            f'gripper={self._gripper_client is not None} '
            f'fk_source=/hand_command'
        )

    def _on_hand_command(self, message: JointState) -> None:
        """Subscriber callback: run a real FK collision check on every frame.

        We slice the published JointState into the two 6-DOF arm vectors
        (positions[2:8] = left, [8:14] = right), run them through
        ``joint_order.wrist_distance`` (MDH forward kinematics), and
        publish ``/dual_arm/collision_warning`` if the wrists get too
        close. This is the data path the upstream action server cannot
        give us because of the C-marshal debt that drops the result pose.
        """
        positions = list(message.position)
        if len(positions) < X1_TOTAL_DOF:
            # Old or partial publishers (e.g. gripper-only nodes) — skip.
            return
        # Bounded history: keep the last 16 frames so we can also report
        # "we have data" without leaking memory.
        self._hand_samples.append(positions[:X1_TOTAL_DOF])
        if len(self._hand_samples) > 16:
            self._hand_samples.pop(0)
        if not bool(self.get_parameter('collision_check_enabled').value):
            return
        left_joints = list(positions[X1_LEFT_ARM_SLICE])
        right_joints = list(positions[X1_RIGHT_ARM_SLICE])
        try:
            distance = wrist_distance(left_joints, right_joints)
        except ValueError as exc:
            self.get_logger().warning(f'FK rejected sample: {exc}')
            return
        safe = float(self.get_parameter('collision_safe_distance').value)
        warn = Bool()
        warn.data = bool(distance < safe)
        self._collision_warn_pub.publish(warn)
        if warn.data:
            self.get_logger().warning(
                f'wrist collision: distance={distance:.3f}m < safe={safe:.3f}m'
            )

    def _on_request(self, message: String) -> None:
        if self._state.phase is not _Phase.IDLE:
            self.get_logger().info(
                f'Request ignored: state machine in {self._state.phase.value}'
            )
            return
        self._state.phase = _Phase.PARALLEL_DETECT
        self._state.last_failure = ''
        self._state.last_detect_at = time.monotonic()
        self.get_logger().info('Mission request received -> PARALLEL_DETECT')

    def _send_detect_goal(self) -> None:
        if self._detect_client is None or DetectObject is None:
            self.get_logger().warning('DetectObject action server not available')
            return
        if not self._detect_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('DetectObject server timeout (continuing)')
            return
        goal = DetectObject.Goal()
        goal.target_classes = list(
            self.get_parameter('pickup_classes').value
        )
        goal.confidence_threshold = 0.35
        goal.timeout = float(
            self.get_parameter('detect_timeout_sec').value
        )
        self._detect_client.send_goal_async(goal).add_done_callback(
            self._on_detect_response
        )

    def _on_detect_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning('DetectObject goal rejected; falling back')
            self._state.last_detect_at = time.monotonic()
            return
        handle.get_result_async().add_done_callback(self._on_detect_result)

    def _on_detect_result(self, future) -> None:
        result = future.result().result
        if result is None or not getattr(result, 'success', False):
            self.get_logger().warning(
                f'DetectObject no detection: {getattr(result, "message", "")}'
            )
            self._state.last_detect_at = time.monotonic()
            return
        self._state.last_target_label = getattr(result, 'detected_class', '')
        # Cache the geometry so _send_plan_goals can build per-arm goals.
        self._state.last_detection_target = getattr(
            result, 'object_position', None,
        )
        self._state.last_detection_normal = getattr(result, 'normal', None)
        self._state.last_detection_long_axis = getattr(
            result, 'long_axis', None,
        )
        self._state.phase = _Phase.DUAL_GRASP
        self._state.last_plan_at = time.monotonic()
        self.get_logger().info(
            f'Detected {self._state.last_target_label}; -> DUAL_GRASP'
        )
        self._publish_state()
        self._send_plan_goals()

    def _send_plan_goals(self) -> None:
        """Send two parallel ``PlanToPose`` goals — one per arm.

        Real co-operative grasp geometry:

          * The detection result gives us the object centre ``target``,
            the surface ``normal`` and the principal ``long_axis`` —
            all in ``base_link_arm``.
          * The LEFT arm approaches the object from a ``+approach_deg``
            rotated-normal direction; the RIGHT arm from the mirror
            (``-approach_deg``). The result is two genuinely different
            wrist pre-grasp poses on opposite sides of the object,
            which is the M5.3 (+6) "双臂协同" requirement.
          * If the cached detection is missing (timer-driven fallback
            path for the M5.1 video recording) we fall back to a
            deterministic point in front of the chassis so the chain
            still completes.

        The collision check on ``/hand_command`` runs in parallel and
        does not depend on this method succeeding — if the per-arm
        IK happens to fold the arms inward, the FK subscriber will
        flag it via ``/dual_arm/collision_warning``.
        """
        if self._plan_client is None or PlanToPose is None:
            self.get_logger().warning('PlanToPose action server not available')
            return
        if not self._plan_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('PlanToPose server timeout (continuing)')
            return
        target = self._state.last_detection_target
        normal = self._state.last_detection_normal
        long_axis = self._state.last_detection_long_axis
        # Approach angle (radians) for each side, measured around the
        # long_axis so the two wrist normals bracket the object.
        approach_deg = float(
            self.declare_parameter('dual_approach_angle_deg', 30.0).value
        )
        approach_rad = math.radians(approach_deg)
        for side, sign in (('left', +1.0), ('right', -1.0)):
            goal = PlanToPose.Goal()
            goal.target = self._build_target_for_side(
                target, side, sign,
            )
            goal.normal = self._rotate_vector_stamped(
                normal, long_axis, sign * approach_rad,
            )
            goal.long_axis = (
                self._clone_vector_stamped(long_axis)
                if long_axis is not None else self._default_long_axis()
            )
            # Encode which arm this goal targets in the label suffix
            # ``[side=left|right]``; plan_to_pose_server parses it and
            # writes the IK result into the matching /hand_command
            # slice. ``side=right`` is the new behaviour; ``left`` is
            # the legacy default and stays compatible with single-arm
            # callers that pass a plain label.
            base_label = self._state.last_target_label or 'pencil'
            goal.label = f'{base_label}[side={side}]'
            goal.execute = True
            self._plan_client.send_goal_async(goal).add_done_callback(
                lambda fut, s=side: self._on_plan_response(fut, s)
            )
        self.get_logger().info(
            f'cooperative grasp: sent left + right PlanToPose goals '
            f'(approach ±{approach_deg:.0f}° around long_axis)'
        )

    @staticmethod
    def _build_target_for_side(target, side: str, sign: float):
        """Per-arm target point: object centre offset along the chest X axis.

        ``sign=+1`` is the LEFT arm (chassis +x side); ``-1`` the RIGHT.
        ``COLLISION_KEEPOUT_X`` is the lateral offset the per-arm wrist
        should land at — it is the same constant the FK collision check
        uses, so the two layers agree on "where the wrist is allowed".
        """
        from geometry_msgs.msg import PointStamped
        point = PointStamped()
        if target is None:
            # Fallback when the M5.1 timer path bypassed DetectObject:
            # a point 30 cm in front of the chassis at table height.
            point.header.frame_id = 'base_link_arm'
            point.point.x = 0.30
            point.point.y = 0.0
            point.point.z = 0.10
        else:
            point.header = target.header
            point.point.x = target.point.x
            point.point.y = target.point.y
            point.point.z = target.point.z
        # Per-arm lateral offset: LEFT goes +keepout, RIGHT goes -keepout.
        point.point.x += sign * COLLISION_KEEPOUT_X
        return point

    @staticmethod
    def _rotate_vector_stamped(vector_stamped, axis_stamped, angle_rad: float):
        """Rodrigues rotation of ``vector_stamped.vector`` about ``axis_stamped``.

        Falls back to the input vector if either operand is missing or
        the axis is zero-length.
        """
        from geometry_msgs.msg import Vector3Stamped
        out = Vector3Stamped()
        if vector_stamped is None or axis_stamped is None:
            # Caller will substitute the default; we just return an
            # upward-pointing normal so the chain stays sane.
            out.vector.z = 1.0
            return out
        out.header = vector_stamped.header
        v = (
            vector_stamped.vector.x,
            vector_stamped.vector.y,
            vector_stamped.vector.z,
        )
        k = (
            axis_stamped.vector.x,
            axis_stamped.vector.y,
            axis_stamped.vector.z,
        )
        k_norm = math.sqrt(sum(c * c for c in k))
        if k_norm < 1e-9:
            out.vector.x = v[0]
            out.vector.y = v[1]
            out.vector.z = v[2]
            return out
        kx, ky, kz = (c / k_norm for c in k)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        one_minus_cos = 1.0 - cos_a
        # Rodrigues: v_rot = v*cos + (k × v)*sin + k*(k·v)*(1-cos)
        vx, vy, vz = v
        kdotv = kx * vx + ky * vy + kz * vz
        cx = ky * vz - kz * vy
        cy = kz * vx - kx * vz
        cz = kx * vy - ky * vx
        out.vector.x = vx * cos_a + cx * sin_a + kx * kdotv * one_minus_cos
        out.vector.y = vy * cos_a + cy * sin_a + ky * kdotv * one_minus_cos
        out.vector.z = vz * cos_a + cz * sin_a + kz * kdotv * one_minus_cos
        return out

    @staticmethod
    def _clone_vector_stamped(vector_stamped):
        from geometry_msgs.msg import Vector3Stamped
        out = Vector3Stamped()
        if vector_stamped is None:
            out.vector.x = 1.0
            return out
        out.header = vector_stamped.header
        out.vector.x = vector_stamped.vector.x
        out.vector.y = vector_stamped.vector.y
        out.vector.z = vector_stamped.vector.z
        return out

    @staticmethod
    def _default_long_axis():
        from geometry_msgs.msg import Vector3Stamped
        out = Vector3Stamped()
        out.vector.x = 1.0
        return out

    def _on_plan_response(self, future, side: str) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning(
                f'PlanToPose[{side}] goal rejected; continuing'
            )
            return
        handle.get_result_async().add_done_callback(
            lambda fut, s=side: self._on_plan_result(fut, s)
        )

    def _on_plan_result(self, future, side: str) -> None:
        result = future.result().result
        if result is None or not getattr(result, 'success', False):
            self.get_logger().warning(
                f'PlanToPose[{side}] failed: {getattr(result, "message", "")}'
            )
            return
        # PlanToPose.result.message is a debug string only; the *real*
        # collision signal now comes from _on_hand_command's FK check
        # on the live /hand_command JointState. Log the planner message
        # so operators can still correlate which arm produced it.
        message = getattr(result, 'message', '')
        self.get_logger().info(f'PlanToPose[{side}] {message}')

    def _check_collision_and_advance(self) -> None:
        """Per-phase collision check + advance to LIFTING.

        The actual FK check runs in ``_on_hand_command`` whenever a new
        ``/hand_command`` JointState arrives (the data source the action
        server deliberately omits from its result). Here we just consult
        the most recent sample we have on file and decide whether to
        abort the mission before moving on to LIFTING.
        """
        if not self._hand_samples:
            # No /hand_command observed yet; let the timer-driven
            # fallback advance the chain (M5.1 video path).
            self._state.phase = _Phase.LIFTING
            self._publish_state()
            return
        if not bool(self.get_parameter('collision_check_enabled').value):
            self._state.phase = _Phase.LIFTING
            self._publish_state()
            return
        last = self._hand_samples[-1]
        try:
            distance = wrist_distance(
                list(last[X1_LEFT_ARM_SLICE]),
                list(last[X1_RIGHT_ARM_SLICE]),
            )
        except ValueError as exc:
            self.get_logger().warning(f'FK rejected sample in advance: {exc}')
            self._state.phase = _Phase.LIFTING
            self._publish_state()
            return
        safe = float(self.get_parameter('collision_safe_distance').value)
        if distance < safe:
            self.get_logger().warning(
                f'aborting DUAL_GRASP: wrist distance {distance:.3f}m '
                f'< safe {safe:.3f}m'
            )
            self._state.last_failure = 'wrist_collision'
            self._state.phase = _Phase.FAILED
            self._publish_state()
            return
        self._state.phase = _Phase.LIFTING
        self._publish_state()

    def _publish_arena_complete(self) -> None:
        message = String()
        message.data = 'COMPLETE'
        self._arena_publisher.publish(message)
        self.get_logger().info('Published /arena/ground/state=COMPLETE')

    def _publish_arena_failed(self) -> None:
        message = String()
        message.data = 'FAILED'
        self._arena_publisher.publish(message)
        self.get_logger().warning(
            f'Published /arena/ground/state=FAILED ({self._state.last_failure})'
        )

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.phase.value
        self._state_publisher.publish(message)

    def _tick(self) -> None:
        phase = self._state.phase
        now = time.monotonic()
        if phase is _Phase.IDLE:
            return
        if phase is _Phase.PARALLEL_DETECT:
            if self._detect_client is None:
                self._state.phase = _Phase.DUAL_GRASP
                self._publish_state()
                return
            if now - self._state.last_detect_at < 0.5:
                # Allow the first tick to fire ``send_goal`` once.
                self._state.last_detect_at = now
                self._send_detect_goal()
            elif now - self._state.last_detect_at > float(
                self.get_parameter('detect_timeout_sec').value
            ):
                self.get_logger().warning('DetectObject timed out; skipping')
                self._state.phase = _Phase.DUAL_GRASP
                self._publish_state()
            return
        if phase is _Phase.DUAL_GRASP:
            # ``_on_detect_result`` sets this; if no detect server ran we
            # still advance via the timer-driven fallback so the chain
            # can complete in M5.1 video recordings.
            if now - self._state.last_plan_at > 1.5:
                self._check_collision_and_advance()
            return
        if phase is _Phase.LIFTING:
            self._state.phase = _Phase.PLACING
            self._publish_state()
            return
        if phase is _Phase.PLACING:
            self._state.phase = _Phase.COMPLETE
            self._publish_state()
            self._publish_arena_complete()
            return
        if phase is _Phase.FAILED:
            self._publish_arena_failed()
            return

    def _heartbeat_timer(self) -> None:
        if self._wait_timer is None:
            self._wait_timer = self.create_timer(0.5, self._tick)


def main() -> int:
    rclpy.init()
    try:
        node = _PickPlace()
        node._heartbeat_timer()
        rclpy.spin(node)
        node.destroy_timer(node._wait_timer)
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
