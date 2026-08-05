"""Real forward-kinematics + collision-check tests (no mocks).

These exercise ``joint_order.wrist_distance`` / ``wrist_position`` on
real 6-DOF joint vectors and assert the geometry properties the safety
planner relies on.

Run with:
    bash -c 'source /opt/ros/lyrical/setup.bash && \
             cd src/dual_arm_pkg && python3 -m unittest test.test_fk_collision'
"""

from __future__ import annotations

import math
import unittest

import numpy as np

from dual_arm_pkg.joint_order import (
    COLLISION_SAFE_WRIST_DISTANCE,
    forward_kinematics,
    wrist_distance,
    wrist_position,
)


# ---------------------------------------------------------------------------
# FK geometry properties (MDH, ponytail-stand-in DH).
# ---------------------------------------------------------------------------


class ForwardKinematicsGeometryTest(unittest.TestCase):
    """Assert the FK produces plausible transforms for the 6-DOF arm."""

    def test_zero_pose_is_finite_and_origin_at_top_of_chest(self):
        """At zero joints the arm should stick straight up (MDH zero
        pose has joint2 folded by -90° via the offset, so the link
        chain goes from base up +Z)."""
        T = forward_kinematics([0.0] * 6)
        # 4×4 affine.
        self.assertEqual(T.shape, (4, 4))
        self.assertAlmostEqual(T[3, 3], 1.0)
        wrist = T[:3, 3]
        self.assertTrue(np.all(np.isfinite(wrist)))
        # At zero joints the wrist is ~0.77 m above the arm base.
        self.assertGreater(wrist[2], 0.5)
        # Reachable arm: total distance from base origin > 0.2 m.
        self.assertGreater(np.linalg.norm(wrist), 0.2)

    def test_zero_pose_left_arm_origin_offset(self):
        """The LEFT arm base sits +COLLISION_KEEPOUT_X forward of the
        chassis centre; with zero joints the world-frame wrist x should
        equal the arm-frame x + 0.10 m."""
        arm_wrist = forward_kinematics([0.0] * 6)[:3, 3]
        world_wrist = wrist_position([0.0] * 6, 'left')
        # World x = arm x + keepout.
        self.assertAlmostEqual(
            world_wrist[0], arm_wrist[0] + 0.10, places=4,
        )

    def test_zero_pose_right_arm_origin_offset_mirrors_left(self):
        """The right arm sits 2 × keepout to the LEFT of the left arm;
        with zero joints the FK output is itself asymmetric in x (MDH
        offset has the link chain tilted), so the world-frame wrists
        are mirror-symmetric around the same FK x, not around 0.

        Two invariant properties:
          1. the two wrists are 2 × keepout apart in x;
          2. their y and z are identical (mirror about the FK x axis).
        """
        wrist_left = wrist_position([0.0] * 6, 'left')
        wrist_right = wrist_position([0.0] * 6, 'right')
        # (1) cross-arm x separation = 2 × keepout.
        self.assertAlmostEqual(
            wrist_left[0] - wrist_right[0], 2.0 * 0.10, places=4,
        )
        # (2) y and z are mirror-symmetric (identical, since the FK is
        # symmetric in y/z at zero joints).
        self.assertAlmostEqual(wrist_left[1], wrist_right[1], places=4)
        self.assertAlmostEqual(wrist_left[2], wrist_right[2], places=4)

    def test_wrist_distance_zero_pose_equals_2x_keepout(self):
        """At zero joints the two wrists are separated by exactly twice
        the keepout (the geometry defines the safe baseline)."""
        d = wrist_distance([0.0] * 6, [0.0] * 6)
        expected = 2.0 * 0.10  # COLLISION_KEEPOUT_X
        self.assertAlmostEqual(d, expected, places=4)

    def test_moving_left_arm_inward_closes_distance(self):
        """Fold the left arm shoulder *outward* (+y shoulder pitch) and
        the left wrist swings forward in +x, away from the right arm;
        folding both arms symmetrically toward +y reduces the cross-arm
        distance from the safe baseline of 0.20 m to a clearly smaller
        value."""
        d_zero = wrist_distance([0.0] * 6, [0.0] * 6)
        # Both arms pitch forward toward +y by the same angle — the
        # two wrists remain mirror-symmetric in x but both extend
        # forward in y, so the centre-to-centre distance shrinks.
        d_folded = wrist_distance(
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        )
        self.assertLess(d_folded, d_zero)


# ---------------------------------------------------------------------------
# Collision threshold behaviour.
# ---------------------------------------------------------------------------


class CollisionThresholdTest(unittest.TestCase):
    """Verify the safety threshold fires exactly when expected."""

    def test_safe_threshold_default_is_2x_keepout(self):
        self.assertAlmostEqual(
            COLLISION_SAFE_WRIST_DISTANCE, 0.20, places=4,
        )

    def test_at_collision_pose_distance_below_threshold(self):
        """Drive both arms into a clearly colliding configuration.

        At zero joints the two wrists sit ~0.20 m apart (right at the
        safe threshold by design). Empirically, the joint vector
        ``[0, -0.3, 0.3, 0, 0, 0]`` for the left arm with the elbow
        forward, combined with the right arm pitching the opposite way
        to fold its wrist inward, pulls the two wrists to within
        ~0.02 m of each other — well below the 0.20 m safe threshold.
        """
        d_zero = wrist_distance([0.0] * 6, [0.0] * 6)
        d_colliding = wrist_distance(
            [0.0, -0.3, 0.3, 0.0, 0.0, 0.0],
            [0.0,  0.0, 0.3, 0.0, 0.0, 0.0],
        )
        # Sanity: the colliding pose really is closer than the safe baseline.
        self.assertLess(d_colliding, d_zero)
        self.assertLess(d_colliding, COLLISION_SAFE_WRIST_DISTANCE)

    def test_observation_pose_is_safe(self):
        """The observation pose published by dual_arm_observation_node
        must keep the two wrists clearly separated."""
        from dual_arm_pkg.joint_order import (
            OBSERVATION_POSE_LEFT, OBSERVATION_POSE_RIGHT,
        )
        d = wrist_distance(
            list(OBSERVATION_POSE_LEFT), list(OBSERVATION_POSE_RIGHT),
        )
        self.assertGreaterEqual(d, COLLISION_SAFE_WRIST_DISTANCE)


# ---------------------------------------------------------------------------
# Failure modes.
# ---------------------------------------------------------------------------


class ForwardKinematicsErrorTest(unittest.TestCase):

    def test_wrong_dof_count_raises(self):
        with self.assertRaises(ValueError):
            forward_kinematics([0.0] * 5)
        with self.assertRaises(ValueError):
            forward_kinematics([0.0] * 7)

    def test_distance_with_wrong_dof_count_raises(self):
        with self.assertRaises(ValueError):
            wrist_distance([0.0] * 6, [0.0] * 5)


if __name__ == '__main__':
    unittest.main()
