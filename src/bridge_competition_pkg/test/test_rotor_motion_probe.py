"""Unit tests for direct-rotor response metrics."""

import math
import unittest

from bridge_competition_pkg.rotor_motion_probe import (
    angular_velocity_extrema,
    quaternion_angle,
    vector_distance,
)


class RotorMotionProbeTest(unittest.TestCase):
    """Cover geometry helpers used by the live response assertion."""

    def test_vector_distance(self):
        """Vector distance uses all axes."""
        self.assertAlmostEqual(vector_distance((0, 0, 0), (3, 4, 0)), 5.0)

    def test_quaternion_angle_ignores_sign(self):
        """Equivalent signed quaternions have zero angular distance."""
        self.assertAlmostEqual(
            quaternion_angle((0, 0, 0, 1), (0, 0, 0, -1)),
            0.0,
        )

    def test_quaternion_angle_for_quarter_turn(self):
        """A 90-degree yaw produces a pi-over-two delta."""
        half_angle = math.pi / 4.0
        self.assertAlmostEqual(
            quaternion_angle(
                (0, 0, 0, 1),
                (0, 0, math.sin(half_angle), math.cos(half_angle)),
            ),
            math.pi / 2.0,
        )

    def test_quaternion_angle_rejects_zero(self):
        """Invalid zero quaternions are rejected explicitly."""
        with self.assertRaises(ValueError):
            quaternion_angle((0, 0, 0, 0), (0, 0, 0, 1))

    def test_angular_velocity_extrema_preserve_axis_signs(self):
        """Rotor mapping evidence includes signed FLU rates on every axis."""
        samples = [
            {'angular_velocity': (0.1, -0.2, 0.3)},
            {'angular_velocity': (-0.4, 0.5, -0.6)},
        ]
        self.assertEqual(
            angular_velocity_extrema(samples),
            {
                'min_angular_velocity_flu_rad_s': (-0.4, -0.2, -0.6),
                'max_angular_velocity_flu_rad_s': (0.1, 0.5, 0.3),
            },
        )

    def test_angular_velocity_extrema_reject_empty_samples(self):
        """An empty pulse cannot produce trustworthy signed evidence."""
        with self.assertRaises(ValueError):
            angular_velocity_extrema([])


if __name__ == '__main__':
    unittest.main()
