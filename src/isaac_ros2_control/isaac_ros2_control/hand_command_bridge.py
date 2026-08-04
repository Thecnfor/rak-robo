#!/usr/bin/env python3
"""Bridge `/hand_command` (16-DOF JointState from dual_arm_pkg) into the two
Isaac Sim / USD-baked graphs that actually drive the X1 robot:

  /joint_command     (sensor_msgs/JointState, 14 DOF)
                     - 2 wheel joints + 6 left arm + 6 right arm
  /gripper_command   (sensor_msgs/JointState, 2 DOF)
                     - gripper_left_joint + gripper_right_joint

The X1 USD scene at
`/var/workspace/docker/isaac/scenes/active/worlds/X1_race/X1_race_scene.usd`
bakes two `ROS_JointStates` OmniGraphs that subscribe to `/joint_command` and
`/gripper_command` respectively (see `scene_app.py` lines 1408-1412). The
dual_arm packages publish a single `/hand_command` JointState with 16
positions in this order:

  wheel_left_joint, wheel_right_joint,
  arm_left_joint_{1..6},
  arm_right_joint_{1..6},
  gripper_left_joint, gripper_right_joint

This node is the only host-side splitter, so the picking/observation
pipeline stays decoupled from how Isaac internally drives the two graphs.

If any of the joint names are missing from the incoming message the node
fills the slot with 0.0 and logs a warning so the bringup never wedges on a
partial JointState.
"""

from __future__ import annotations

import sys
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 14 DOF arm+wheel group (2 + 6 + 6) — order MUST match the
# `ROS_JointStates` USD graph joint_names in scene_app.py + the USD scene.
# X1_race_scene.usd defines the actual robot joints as ``joint{N}_L`` /
# ``joint{N}_R`` (6 per arm), plus 2 wheel joints and 4 gripper fingers.
# The dual_arm packages publish ``/hand_command`` with the legacy
# ``arm_left_joint_N`` / ``arm_right_joint_N`` names; this bridge
# translates them to the names the USD graph subscribes to.
_JOINT_NAMES: List[str] = [
    'front_left_wheel',
    'front_right_wheel',
    'joint1_L',
    'joint2_L',
    'joint3_L',
    'joint4_L',
    'joint5_L',
    'joint6_L',
    'joint1_R',
    'joint2_R',
    'joint3_R',
    'joint4_R',
    'joint5_R',
    'joint6_R',
]
# X1 USD has 4 finger joints; the ROS_JointStates_01 graph exposes the
# full set, the dual_arm packages use a single ``gripper_*_joint`` slot
# (the gripper_server opens/closes them in sync).
_GRIPPER_NAMES: List[str] = [
    'left_gripper_left_joint2',
    'left_gripper_right_join2',
    'right_gripper_left_joint2',
    'right_gripper_right_join2',
]

# Aliases for incoming ``/hand_command`` joint names (legacy dual_arm
# names) — the bridge selects positions by these aliases and emits them
# under the X1 USD names on ``/joint_command``.
_ARM_LEFT_ALIASES: List[str] = [
    'arm_left_joint_1',
    'arm_left_joint_2',
    'arm_left_joint_3',
    'arm_left_joint_4',
    'arm_left_joint_5',
    'arm_left_joint_6',
]
_ARM_RIGHT_ALIASES: List[str] = [
    'arm_right_joint_1',
    'arm_right_joint_2',
    'arm_right_joint_3',
    'arm_right_joint_4',
    'arm_right_joint_5',
    'arm_right_joint_6',
]
_WHEEL_ALIASES: List[str] = [
    'wheel_left_joint',
    'wheel_right_joint',
]
_GRIPPER_ALIASES: List[str] = [
    'gripper_left_joint',
    'gripper_right_joint',
]


def _select_positions(
    msg: JointState,
    wanted: Iterable[str],
    fallback_total: int,
) -> List[float]:
    """Project the 16-DOF ``/hand_command`` onto ``wanted`` joint names.

    Matching is by name; if a name is missing, the slot is filled with 0.0
    and a warning is emitted by the caller. If the incoming message is
    shorter than ``fallback_total``, remaining slots are zero-padded so the
    ROS graphs always receive a complete command.
    """
    name_to_position = dict(zip(msg.name, msg.position))
    out: List[float] = []
    for joint_name in wanted:
        if joint_name in name_to_position:
            out.append(float(name_to_position[joint_name]))
        else:
            out.append(0.0)
    return out


class HandCommandBridge(Node):
    """Split ``/hand_command`` → ``/joint_command`` + ``/gripper_command``."""

    def __init__(self) -> None:
        super().__init__('hand_command_bridge')
        self._subscription = self.create_subscription(
            JointState, '/hand_command', self._on_hand_command, 10,
        )
        self._joint_pub = self.create_publisher(
            JointState, '/joint_command', 10,
        )
        self._gripper_pub = self.create_publisher(
            JointState, '/gripper_command', 10,
        )
        self._latched: JointState = JointState()
        self.get_logger().info(
            'hand_command_bridge ready: /hand_command -> '
            f'{len(_JOINT_NAMES)} DOF joint + {len(_GRIPPER_NAMES)} DOF gripper'
        )

    def _on_hand_command(self, msg: JointState) -> None:
        # Map legacy ``/hand_command`` names → X1 USD joint names.
        alias_positions = dict(zip(msg.name, msg.position))
        joint_positions: List[float] = []
        joint_aliases: List[str] = []
        for alias in (
            _WHEEL_ALIASES + _ARM_LEFT_ALIASES + _ARM_RIGHT_ALIASES
        ):
            if alias in alias_positions:
                joint_positions.append(float(alias_positions[alias]))
                joint_aliases.append(alias)
            else:
                joint_positions.append(0.0)
                joint_aliases.append(alias)
        # Gripper: ``gripper_left_joint``/``gripper_right_joint`` are
        # broadcast to the four X1 finger joints so the gripper tracks
        # the requested aperture on both sides.
        gripper_positions: List[float] = []
        gripper_aliases: List[str] = []
        for alias in _GRIPPER_ALIASES:
            if alias in alias_positions:
                gripper_positions.append(float(alias_positions[alias]))
                gripper_aliases.append(alias)
            else:
                gripper_positions.append(0.0)
                gripper_aliases.append(alias)
        # Surface any missing names so the operator can re-align joint
        # ordering instead of having the X1 silently sit at zero pose.
        missing = [
            alias for alias in (
                _WHEEL_ALIASES + _ARM_LEFT_ALIASES + _ARM_RIGHT_ALIASES
            )
            if alias not in alias_positions
        ]
        if missing:
            self.get_logger().warn(
                f'joint_command missing aliases={missing}; zero-padding'
            )
        missing_gripper = [
            alias for alias in _GRIPPER_ALIASES
            if alias not in alias_positions
        ]
        if missing_gripper:
            self.get_logger().warn(
                f'gripper_command missing aliases={missing_gripper}; '
                'zero-padding'
            )

        joint_msg = JointState()
        joint_msg.header = msg.header
        joint_msg.name = list(_JOINT_NAMES)
        joint_msg.position = joint_positions
        self._joint_pub.publish(joint_msg)

        gripper_msg = JointState()
        gripper_msg.header = msg.header
        gripper_msg.name = list(_GRIPPER_NAMES)
        gripper_msg.position = gripper_positions
        self._gripper_pub.publish(gripper_msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = HandCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())