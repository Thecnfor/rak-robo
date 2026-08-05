"""Unit tests for the code-first ROS 2 graph builder.

These tests focus on the *spec* construction and the pure-Python
helpers (pose math, side parsing) — the OmniGraph edits themselves
require a live Isaac Sim so they live in scene_app.py integration
tests rather than here.

Run:
    bash -c 'source /opt/ros/lyrical/setup.bash && \
             cd src/dual_arm_pkg && python3 -m unittest test.test_x1_graph_builder'
"""

from __future__ import annotations

import unittest

from dual_arm_pkg.x1_graph_builder import (
    DiffDriveSpec,
    JointSpec,
    LidarSpec,
    CameraSpec,
    RobotGraphSpec,
    x1_spec,
)


class SpecConstructionTest(unittest.TestCase):
    def test_joint_spec_defaults_topic(self):
        spec = JointSpec('arm_left_joint_1')
        self.assertEqual(spec.topic, '/joint_command')

    def test_diff_drive_spec_round_trips(self):
        spec = DiffDriveSpec(
            left_wheel_joint='wheel_left_joint',
            right_wheel_joint='wheel_right_joint',
            wheel_radius=0.0675,
            wheel_base=0.233,
        )
        self.assertEqual(spec.wheel_radius, 0.0675)
        self.assertEqual(spec.wheel_base, 0.233)


class X1SpecTest(unittest.TestCase):
    def test_x1_spec_covers_all_joint_groups(self):
        spec = x1_spec('/World/mercury_x1', domain_id=45)
        # 2 wheels + 6 left + 6 right = 14 arm_joints
        self.assertEqual(len(spec.joints), 14)
        self.assertEqual(len(spec.gripper_joints), 2)
        self.assertEqual(len(spec.cameras), 2)
        self.assertEqual(len(spec.lidars), 1)
        self.assertIsNotNone(spec.diff_drive)

    def test_x1_spec_grippers_get_their_own_topic(self):
        spec = x1_spec('/World/mercury_x1')
        for j in spec.gripper_joints:
            self.assertEqual(j.topic, '/gripper_command')
        for j in spec.joints:
            self.assertEqual(j.topic, '/joint_command')

    def test_x1_spec_domain_id_propagates(self):
        spec = x1_spec('/World/mercury_x1', domain_id=99)
        self.assertEqual(spec.domain_id, 99)
        # Sanity: the dual camera + lidar topics are kept stable
        # across domain changes.
        self.assertEqual(spec.cameras[0].rgb_topic, '/rgb')


class IdempotencyContractTest(unittest.TestCase):
    """The graph builder must be safe to call twice in a row."""

    def test_x1_spec_serializes_to_dict(self):
        # Smoke test that the spec is dataclass-round-trippable; the
        # idempotency itself is enforced inside the OmniGraph helpers
        # via _make_or_replace.
        spec = x1_spec('/World/mercury_x1')
        from dataclasses import asdict
        d = asdict(spec)
        self.assertEqual(d['domain_id'], 45)
        self.assertEqual(d['robot_prim'], '/World/mercury_x1')


if __name__ == '__main__':
    unittest.main()
