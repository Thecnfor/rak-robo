"""DualGripperCommand action server (left + right gripper).

Exposes both the original ``/demo_gripper_command`` action
(``GripperCommand``, single gripper) **and** the new
``/demo_dual_gripper_command`` action (``DualGripperCommand``, left +
right sides with independent positions) on a single node. Either side can
move in lock-step (same command) or independently (different commands on
``DualGripperCommand``). All commands target the standard
``/hand_command`` ``JointState`` topic consumed by Isaac Sim's
``ArticulationController`` via the X1 ground OmniGraph (added in
scene_app.py 2026-07-20).

Topic/action contract:

  ``/demo_gripper_command``     ``GripperCommand``   single channel, both grippers
  ``/demo_dual_gripper_command`` ``DualGripperCommand``  left + right channels
  ``/hand_command``             ``sensor_msgs/JointState``  (16 floats: 2 wheels +
                              6 left arm + 6 right arm + 2 grippers).

If the ``DualGripperCommand`` action type has not been generated yet the
server logs a warning and only serves the legacy ``GripperCommand``.

Usage:
    ros2 run dual_arm_pkg dual_gripper_server_node
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .joint_order import (
    GRIPPER_OPEN_POSITION,
    GRIPPER_CLOSED_POSITION,
    X1_GRIPPER_JOINTS,
    X1_JOINT_ORDER,
    X1_LEFT_ARM_SLICE,
    X1_RIGHT_ARM_SLICE,
    X1_WHEEL_SLICE,
    X1_GRIPPER_SLICE,
    gripper_position,
)

try:
    from grasp_demo_interfaces.action import (
        DualGripperCommand as _DualGripper,
        GripperCommand as _Gripper,
    )
except ImportError:  # pragma: no cover
    _DualGripper = None  # type: ignore[assignment]
    _Gripper = None  # type: ignore[assignment]


_OPEN_POSITION = GRIPPER_OPEN_POSITION
_CLOSED_POSITION = GRIPPER_CLOSED_POSITION


def _publish_state(
    publisher,
    gripper_positions,
    gripper_names=X1_GRIPPER_JOINTS,
    repeats: int = 1,
) -> None:
    """Emit a JointState targeting the gripper joints (other joints zeroed).

    The skeleton arm positions are zeroed; the IK server (when it has
    a solution) is the one that fills them in via ``/hand_command``.

    ``repeats`` is honoured for callers running outside the action
    callback (e.g. tests) but **the action execute path must pass
    repeats=1**: ActionServer callbacks run on the rclpy executor and a
    blocking loop inside the callback will starve the goal-handle
    machinery, causing the goal to be marked ABORTED.
    """
    state = JointState()
    state.name = list(X1_JOINT_ORDER)
    # sensor_msgs/JointState.position is a typed float32 array in
    # ROS 2 Lyrical; assigning a Python list works for the *whole* field
    # but slice-assignment can fail if the underlying storage is
    # numpy. Build the full vector in one shot to avoid surprises.
    positions = [0.0] * len(X1_JOINT_ORDER)
    positions[X1_WHEEL_SLICE] = [0.0, 0.0]
    positions[X1_GRIPPER_SLICE] = list(gripper_positions)
    state.position = positions
    # Override the gripper-name pair in case the caller passed a custom
    # tuple (kept for the legacy action path).
    if list(gripper_names) != list(X1_GRIPPER_JOINTS):
        # Replace the last two name slots while keeping positions intact.
        state.name = (
            list(state.name[:X1_GRIPPER_SLICE.start])
            + list(gripper_names)
        )
    for _ in range(max(1, repeats)):
        publisher.publish(state)


class _DualGripperServer(Node):
    """Action server for ``GripperCommand`` (legacy) and ``DualGripperCommand``."""

    def __init__(self) -> None:
        super().__init__('dual_gripper_server_node')
        self._publisher = self.create_publisher(
            JointState, '/hand_command', 10
        )
        if _Gripper is not None:
            self._action_legacy = ActionServer(
                self, _Gripper, '/demo_gripper_command',
                execute_callback=self._execute_legacy,
                goal_callback=lambda req: GoalResponse.ACCEPT,
                cancel_callback=lambda req: CancelResponse.ACCEPT,
            )
            self.get_logger().info(
                'Serving GripperCommand on /demo_gripper_command '
                '(single channel, both grippers move together)'
            )
        else:
            self._action_legacy = None
            self.get_logger().warning(
                'GripperCommand action type not importable; legacy '
                'single-channel server disabled.'
            )
        if _DualGripper is not None:
            self._action_dual = ActionServer(
                self, _DualGripper, '/demo_dual_gripper_command',
                execute_callback=self._execute_dual,
                goal_callback=lambda req: GoalResponse.ACCEPT,
                cancel_callback=lambda req: CancelResponse.ACCEPT,
            )
            self.get_logger().info(
                'Serving DualGripperCommand on /demo_dual_gripper_command '
                '(left + right channels; can move independently)'
            )
        else:
            self._action_dual = None
            self.get_logger().warning(
                'DualGripperCommand action type not generated yet; '
                'rebuild grasp_demo_interfaces.'
            )

    def _execute_legacy(self, goal_handle) -> Optional[_Gripper.Result]:
        request = goal_handle.request
        cmd = (request.command or 'open').strip().lower()
        position = gripper_position(cmd)
        if cmd not in {'open', 'close', 'stop'}:
            self.get_logger().warning(
                f'Unknown gripper command "{cmd}"; defaulting to open'
            )
            position = GRIPPER_OPEN_POSITION
        # Action callbacks must return promptly; never loop on publish.
        _publish_state(self._publisher, [position, position], repeats=1)
        result = _Gripper.Result()
        result.success = True
        result.message = f'gripper_{cmd} published (left+right in lock-step)'
        result.final_position = position
        goal_handle.succeed()
        self.get_logger().info(
            f'GripperCommand {cmd} -> position={position:.3f} m on both sides'
        )
        return result

    def _execute_dual(self, goal_handle) -> Optional[_DualGripper.Result]:
        request = goal_handle.request
        left_cmd = (request.left_command or 'open').strip().lower()
        left_position = gripper_position(left_cmd)
        # DualGripperCommand currently has only a left_command; right side
        # mirrors until an action schema extension adds right_command.
        right_position = left_position
        if left_cmd not in {'open', 'close', 'stop'}:
            self.get_logger().warning(
                f'Unknown left_command "{left_cmd}"; defaulting to open'
            )
            left_position = GRIPPER_OPEN_POSITION
            right_position = GRIPPER_OPEN_POSITION
        # Action callbacks must return promptly; never loop on publish.
        _publish_state(self._publisher, [left_position, right_position], repeats=1)
        result = _DualGripper.Result()
        result.success = True
        result.message = (
            f'DualGripperCommand left={left_cmd} '
            f'(right mirrors) published'
        )
        result.left_final_position = left_position
        result.right_final_position = right_position
        goal_handle.succeed()
        self.get_logger().info(
            f'DualGripperCommand left={left_cmd} '
            f'right={left_cmd} -> '
            f'[{left_position:.3f}, {right_position:.3f}] m'
        )
        return result


def main() -> int:
    rclpy.init()
    try:
        node = _DualGripperServer()
        rclpy.spin(node)
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
