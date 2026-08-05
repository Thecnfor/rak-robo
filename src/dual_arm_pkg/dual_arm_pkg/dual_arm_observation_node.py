"""Drive the Mercury X1 dual-arm to the observation pose.

The teaching pipeline expects `/hand_command` to receive a 14-DOF `JointState`
(2 wheels + 6 left arm + 6 right arm + 2 gripper positions) before any
perception node publishes. This node publishes a single static observation
pose at startup and on `/hand_command` latching; the rest of the chain can
treat the arms as "ready" without depending on Isaac Sim playback.

The actual joint names are TBD and must be aligned with the prim paths
emitted by `x1_joint_inspector.py` (see `docs/runbooks/M1_modeling_runbook.md`).
For now we publish the *contract* (14 floats + JointState) and let the
Isaac OmniGraph side remap the channel to its actual ArticulationController.

Usage:
    ros2 run dual_arm_pkg dual_arm_observation_node
    ros2 run dual_arm_pkg dual_arm_observation_node --ros-args \
        -p arm_joint_names:='[arm_left_1, ..., gripper_right]'
"""

from __future__ import annotations

import sys
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .joint_order import (
    GRIPPER_NEUTRAL_POSITION,
    OBSERVATION_POSE_LEFT,
    OBSERVATION_POSE_RIGHT,
    X1_JOINT_ORDER,
    X1_LEFT_ARM_JOINTS,
    X1_LEFT_ARM_SLICE,
    X1_RIGHT_ARM_JOINTS,
    X1_WHEEL_SLICE,
    X1_GRIPPER_SLICE,
)


class _ObservationPublisher(Node):
    def __init__(self) -> None:
        super().__init__('dual_arm_observation_node')
        self._publisher = self.create_publisher(
            JointState, '/hand_command', 10)
        joint_names = self._resolve_joint_names()
        self._joint_state = self._build_state(joint_names)
        # Publish once on startup, then latch every 2 s so late-joining
        # subscribers (and re-Discoveries) always see the latest pose.
        self._timer = self.create_timer(2.0, self._publish)

    def _resolve_joint_names(self) -> List[str]:
        # Single source of truth lives in joint_order.X1_JOINT_ORDER; the
        # parameter is exposed only for backwards compatibility with the
        # original launch-file override path.
        declared = self.declare_parameter(
            'arm_joint_names', list(X1_JOINT_ORDER),
        )
        return list(declared.value)

    def _build_state(self, joint_names: List[str]) -> JointState:
        # Use the shared slice indices so wheels / left arm / right arm /
        # gripper stay in lockstep with joint_order.X1_JOINT_ORDER.
        positions = [0.0] * len(X1_JOINT_ORDER)
        positions[X1_WHEEL_SLICE] = [0.0, 0.0]
        positions[X1_LEFT_ARM_SLICE] = list(OBSERVATION_POSE_LEFT)
        positions[X1_LEFT_ARM_SLICE.stop:X1_LEFT_ARM_SLICE.stop + 6] = list(
            OBSERVATION_POSE_RIGHT
        )
        positions[X1_GRIPPER_SLICE] = [
            GRIPPER_NEUTRAL_POSITION, GRIPPER_NEUTRAL_POSITION,
        ]
        if len(joint_names) != len(positions):
            self.get_logger().warning(
                f'joint_names ({len(joint_names)}) does not match positions '
                f'({len(positions)}); using positions in declared order.')
        state = JointState()
        state.header = Header()
        state.name = joint_names
        state.position = [float(v) for v in positions]
        return state

    def _publish(self) -> None:
        self._joint_state.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._joint_state)
        self.get_logger().debug(
            f'Published observation pose on /hand_command '
            f'({len(self._joint_state.position)} joints)')


def main() -> int:
    rclpy.init()
    try:
        node = _ObservationPublisher()
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
