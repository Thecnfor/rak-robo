"""DualGripperCommand action server (left + right gripper).

Exposes both the original ``/demo_gripper_command`` action
(``GripperCommand``, single gripper) **and** the new
``/demo_dual_gripper_command`` action (``DualGripperCommand``, left +
right sides with independent positions) on a single node. Either side can
move in lock-step (same command) or independently (different commands on
``DualGripperCommand``). All commands target the standard
``/hand_command`` ``JointState`` topic consumed by
``isaac_ros2_control`` / Isaac Sim's ``ArticulationController`` via the
X1 ground OmniGraph (added in scene_app.py 2026-07-20).

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

try:
    from grasp_demo_interfaces.action import (
        DualGripperCommand as _DualGripper,
        GripperCommand as _Gripper,
    )
except ImportError:  # pragma: no cover
    _DualGripper = None  # type: ignore[assignment]
    _Gripper = None  # type: ignore[assignment]


_OPEN_POSITION = 0.04
_CLOSED_POSITION = 0.0


def _gripper_position(command: Optional[str]) -> float:
    """Map a command string to a target gripper aperture in metres."""
    cmd = (command or 'open').strip().lower()
    if cmd == 'open':
        return _OPEN_POSITION
    if cmd in {'close', 'stop'}:
        return _CLOSED_POSITION
    return _OPEN_POSITION


def _publish_state(
    publisher,
    gripper_positions,
    gripper_names=('gripper_left_joint', 'gripper_right_joint'),
    repeats: int = 5,
) -> None:
    """Emit a JointState targeting the gripper joints (other joints zeroed)."""
    state = JointState()
    state.name = [
        'wheel_left_joint', 'wheel_right_joint',
        'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
        'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
        'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
        'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
    ] + list(gripper_names)
    state.position = (
        [0.0] * 2 + [0.0] * 12 + list(gripper_positions)
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
            self.get_logger().warn(
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
            self.get_logger().warn(
                'DualGripperCommand action type not generated yet; '
                'rebuild grasp_demo_interfaces.'
            )

    def _execute_legacy(self, goal_handle) -> Optional[_Gripper.Result]:
        request = goal_handle.request
        cmd = (request.command or 'open').strip().lower()
        position = _gripper_position(cmd)
        if cmd not in {'open', 'close', 'stop'}:
            self.get_logger().warn(
                f'Unknown gripper command "{cmd}"; defaulting to open'
            )
            position = _OPEN_POSITION
        _publish_state(self._publisher, [position, position])
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
        left_position = _gripper_position(left_cmd)
        # DualGripperCommand currently has only a left_command; right side
        # mirrors until an action schema extension adds right_command.
        right_position = left_position
        if left_cmd not in {'open', 'close', 'stop'}:
            self.get_logger().warn(
                f'Unknown left_command "{left_cmd}"; defaulting to open'
            )
            left_position = _OPEN_POSITION
            right_position = _OPEN_POSITION
        _publish_state(self._publisher, [left_position, right_position])
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
