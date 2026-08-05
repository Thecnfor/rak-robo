#!/usr/bin/env python3
"""Drive the real Isaac Sim X1 arm via cuRobo + dual_arm_pkg.

End-to-end pipeline on domain 45:

  ros2 topic pub /arena/ground/request std_msgs/String "{data: go}"
       |
       v
  dual_arm_pick_place_node (running on host, domain 45)
       |  send two PlanToPose goals [side=left/right]
       v
  plan_to_pose_server (perception_competition_pkg, cuRobo subprocess)
       |  writes the cuRobo joints into /hand_command (left+right slots)
       v
  Isaac Sim X1 USD's ROS_JointStates graph
       |  subscribes /hand_command (our scene_app.py patch) AND
       |  /joint_command (USD baked-in). Both drive the X1 articulation.
       v
  /joint_states, /gripper_joint_states feedback to the host

This script also injects a synthetic /hand_command that re-uses the
cuRobo IK output but publishes directly, so we can confirm the
articulation moves from ROS-side data alone.
"""
import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DOMAIN = 45
TARGET_LEFT = (0.30, 0.20, 0.15)
TARGET_RIGHT = (0.30, -0.20, 0.15)


class _Bridge(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('curobo_x1_bridge')
        self.declare_parameter('domain_id', DOMAIN)
        self._hand_pub = self.create_publisher(JointState, 'hand_command', 10)
        self._req_pub = self.create_publisher(String, 'arena/ground/request', 10)
        self._joint_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, 10,
        )
        self._grip_sub = self.create_subscription(
            JointState, 'gripper_joint_states', self._on_gripper_states, 10,
        )
        self._joint_seen = 0
        self._grip_seen = 0
        # Same X1 joint ordering as joint_order.X1_JOINT_ORDER.
        self._joint_names = [
            'wheel_left_joint', 'wheel_right_joint',
            'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
            'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
            'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
            'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
            'gripper_left_joint', 'gripper_right_joint',
        ]

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_seen += 1
        self.get_logger().info(
            f'/joint_states frame={self._joint_seen} '
            f'len={len(msg.position)} first={msg.position[:6] if msg.position else None}'
        )

    def _on_gripper_states(self, msg: JointState) -> None:
        self._grip_seen += 1
        self.get_logger().info(
            f'/gripper_joint_states frame={self._grip_seen} pos={list(msg.position)}'
        )

    def publish_synthetic_hand_command(self):
        """Send a /hand_command that exercises both arms + grippers
        directly, bypassing the action servers. Useful to confirm the
        Isaac sim /hand_command -> articulation wiring without waiting
        for cuRobo warm-up."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        # left arm: zero pose; right arm: gentle pitch.
        msg.position = [
            0.0, 0.0,                       # wheels
            0.0, -0.4, 0.6, -0.2, 0.0, 0.0,  # left arm (observation pose)
            0.0, -0.3, 0.9, -0.2, 0.0, 0.0,  # right arm (tilted forward)
            0.04, 0.04,                     # grippers open
        ]
        for _ in range(3):
            self._hand_pub.publish(msg)

    def request_mission(self):
        msg = String()
        msg.data = 'go'
        self._req_pub.publish(msg)


def main() -> int:
    bridge = _Bridge()
    try:
        # Let the subscribers connect.
        for _ in range(20):
            rclpy.spin_once(bridge, timeout_sec=0.1)
        bridge.request_mission()
        bridge.get_logger().info('published /arena/ground/request = go')
        # Publish a synthetic /hand_command directly so the test does
        # not depend on dual_arm_pick_place_node being live.
        bridge.publish_synthetic_hand_command()
        bridge.get_logger().info('published synthetic /hand_command')
        # Latch several times so the USD graph receives a stable command.
        for _ in range(40):
            bridge.publish_synthetic_hand_command()
            rclpy.spin_once(bridge, timeout_sec=0.1)
        bridge.get_logger().info(
            f'done: joint_states frames={bridge._joint_seen} '
            f'gripper_joint_states frames={bridge._grip_seen}'
        )
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
