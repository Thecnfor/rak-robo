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


# 12-DOF observation pose: 6 left + 6 right (rad). Tuned to keep the arms
# clear of the cargo bay and the down-camera frame. Mirrors the values in
# `grasp_demo_pkg/demo_params.yaml::observation_pose` so the two demos
# agree on what "ready" means.
OBSERVATION_POSE_LEFT = [
    0.0,  # arm_left_1  base yaw
    -0.4,  # arm_left_2  shoulder pitch
    0.6,  # arm_left_3  elbow
    -0.2,  # arm_left_4  wrist 1
    0.0,  # arm_left_5  wrist 2
    0.0,  # arm_left_6  wrist 3
]
OBSERVATION_POSE_RIGHT = [p for p in OBSERVATION_POSE_LEFT]


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
        # Two wheel joints + 6 left + 6 right + 2 grippers. The order must
        # match the JointState `position` vector below; any change here
        # requires updating `demo_params.yaml::observation_pose` too.
        declared = self.declare_parameter(
            'arm_joint_names',
            [
                'wheel_left_joint', 'wheel_right_joint',
                'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
                'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
                'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
                'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
                'gripper_left_joint', 'gripper_right_joint',
            ],
        )
        return list(declared.value)

    def _build_state(self, joint_names: List[str]) -> JointState:
        # left arm + right arm pose; wheels at 0; grippers half-open.
        positions = (
            [0.0, 0.0]  # wheels
            + OBSERVATION_POSE_LEFT
            + OBSERVATION_POSE_RIGHT
            + [0.02, 0.02]  # grippers half-open (m)
        )
        if len(joint_names) != len(positions):
            self.get_logger().warn(
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
