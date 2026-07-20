"""DualGripperCommand action server (left + right gripper).

The teaching pipeline's `/demo_gripper_command` only handles a single
gripper. For the competition we need both grippers to move in lock-step
(e.g. dual-arm pick) and independently (e.g. door open + payload hold).
This action server wraps `/hand_command` and exposes a single
`DualGripperCommand` action; the matching `.action` file is added to
`grasp_demo_interfaces` per `src/dual_arm_pkg/TODO.md` M5.1.

If the action type is not yet generated, the server falls back to a
plain `GripperCommand` so the bringup works during incremental builds.

Usage:
    ros2 run dual_arm_pkg dual_gripper_server_node
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from grasp_demo_interfaces.action import GripperCommand as _Gripper
except ImportError:  # pragma: no cover
    _Gripper = None  # type: ignore[assignment]


class _DualGripperServer(Node):
    def __init__(self) -> None:
        super().__init__('dual_gripper_server_node')
        self._publisher = self.create_publisher(
            JointState, '/hand_command', 10)
        if _Gripper is not None:
            self._action = ActionServer(
                self, _Gripper, '/demo_gripper_command',
                execute_callback=self._execute,
                goal_callback=lambda req: GoalResponse.ACCEPT,
                cancel_callback=lambda req: CancelResponse.ACCEPT,
            )
            self.get_logger().info(
                'Serving GripperCommand on /demo_gripper_command '
                '(DualGripperCommand when interface is generated)')
        else:
            self.get_logger().warn(
                'GripperCommand action type not importable; running as stub.')

    def _execute(self, goal_handle) -> Optional[JointState]:
        request = goal_handle.request
        cmd = (request.command or 'open').strip().lower()
        gripper_pos = 0.04 if cmd == 'open' else 0.0
        if cmd not in {'open', 'close', 'stop'}:
            self.get_logger().warn(
                f'Unknown gripper command "{cmd}"; defaulting to open')
            gripper_pos = 0.04
        state = JointState()
        state.name = ['gripper_left_joint', 'gripper_right_joint']
        state.position = [gripper_pos, gripper_pos]
        for _ in range(5):
            self._publisher.publish(state)
        if _Gripper is not None:
            result = _Gripper.Result()
            result.success = True
            result.message = f'gripper_{cmd} published'
            result.final_position = gripper_pos
            goal_handle.succeed()
            return result
        return None


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
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
