"""Real cooperative-grasp geometry tests (no mocks).

These cover ``_PickPlace._build_target_for_side`` and
``_PickPlace._rotate_vector_stamped`` — the two helpers that turn a
single ``DetectObject`` result into two genuinely different
``PlanToPose`` goals for the left and right arms.

Run with:
    bash -c 'source /opt/ros/lyrical/setup.bash && \
             cd src/dual_arm_pkg && python3 -m unittest test.test_cooperative_grasp'
"""

from __future__ import annotations

import math
import unittest


def _pick_class():
    """Lazy import: the pick_place node pulls in rclpy + numpy at top
    level, so we only load it when rclpy is on PYTHONPATH."""
    import rclpy  # noqa: F401
    from dual_arm_pkg.dual_arm_pick_place_node import _PickPlace
    return _PickPlace


try:
    import rclpy  # noqa: F401
    _HAVE_RCLPY = True
except ImportError:
    _HAVE_RCLPY = False


def _point(x: float, y: float, z: float):
    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import Header
    p = PointStamped()
    p.header = Header()
    p.header.frame_id = 'base_link_arm'
    p.point.x = x
    p.point.y = y
    p.point.z = z
    return p


def _vec(x: float, y: float, z: float):
    from geometry_msgs.msg import Vector3Stamped
    v = Vector3Stamped()
    v.vector.x = x
    v.vector.y = y
    v.vector.z = z
    return v


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class CooperativeGraspTargetTest(unittest.TestCase):
    """The two per-arm targets must straddle the object centre on the
    chassis X axis by exactly one ``COLLISION_KEEPOUT_X`` each."""

    def test_left_target_is_plus_keepout_on_x(self):
        cls = _pick_class()
        obj = _point(0.30, 0.0, 0.10)
        left = cls._build_target_for_side(obj, 'left', +1.0)
        from dual_arm_pkg.joint_order import COLLISION_KEEPOUT_X
        self.assertAlmostEqual(left.point.x, obj.point.x + COLLISION_KEEPOUT_X, places=6)
        self.assertAlmostEqual(left.point.y, obj.point.y, places=6)
        self.assertAlmostEqual(left.point.z, obj.point.z, places=6)

    def test_right_target_is_minus_keepout_on_x(self):
        cls = _pick_class()
        obj = _point(0.30, 0.0, 0.10)
        right = cls._build_target_for_side(obj, 'right', -1.0)
        from dual_arm_pkg.joint_order import COLLISION_KEEPOUT_X
        self.assertAlmostEqual(right.point.x, obj.point.x - COLLISION_KEEPOUT_X, places=6)
        self.assertAlmostEqual(right.point.y, obj.point.y, places=6)
        self.assertAlmostEqual(right.point.z, obj.point.z, places=6)

    def test_left_and_right_separation_is_2x_keepout(self):
        cls = _pick_class()
        obj = _point(0.30, 0.0, 0.10)
        left = cls._build_target_for_side(obj, 'left', +1.0)
        right = cls._build_target_for_side(obj, 'right', -1.0)
        from dual_arm_pkg.joint_order import COLLISION_KEEPOUT_X
        self.assertAlmostEqual(
            left.point.x - right.point.x, 2.0 * COLLISION_KEEPOUT_X, places=6,
        )

    def test_no_target_falls_back_to_default_pose(self):
        cls = _pick_class()
        left = cls._build_target_for_side(None, 'left', +1.0)
        # Should produce a sane point near the chassis, not crash.
        self.assertEqual(left.header.frame_id, 'base_link_arm')
        self.assertGreater(left.point.x, 0.0)


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class CooperativeGraspRotationTest(unittest.TestCase):
    """Rodrigues rotation around long_axis must yield two distinct normals."""

    def test_zero_angle_is_identity(self):
        cls = _pick_class()
        normal = _vec(0.0, 0.0, 1.0)  # pointing up
        long_axis = _vec(1.0, 0.0, 0.0)  # rotation axis = +x
        rotated = cls._rotate_vector_stamped(normal, long_axis, 0.0)
        self.assertAlmostEqual(rotated.vector.x, 0.0, places=6)
        self.assertAlmostEqual(rotated.vector.y, 0.0, places=6)
        self.assertAlmostEqual(rotated.vector.z, 1.0, places=6)

    def test_plus_30_and_minus_30_yield_opposite_y_components(self):
        cls = _pick_class()
        normal = _vec(0.0, 0.0, 1.0)
        long_axis = _vec(1.0, 0.0, 0.0)
        plus = cls._rotate_vector_stamped(normal, long_axis, math.radians(30))
        minus = cls._rotate_vector_stamped(normal, long_axis, math.radians(-30))
        # Around the +x axis the y/z components flip sign.
        self.assertAlmostEqual(plus.vector.y, -minus.vector.y, places=6)
        self.assertAlmostEqual(plus.vector.z, minus.vector.z, places=6)
        # Magnitude preserved.
        plus_norm = math.sqrt(plus.vector.x ** 2 + plus.vector.y ** 2 + plus.vector.z ** 2)
        minus_norm = math.sqrt(minus.vector.x ** 2 + minus.vector.y ** 2 + minus.vector.z ** 2)
        self.assertAlmostEqual(plus_norm, 1.0, places=6)
        self.assertAlmostEqual(minus_norm, 1.0, places=6)

    def test_missing_operands_fall_back_to_up_vector(self):
        cls = _pick_class()
        rotated = cls._rotate_vector_stamped(None, None, 0.0)
        self.assertAlmostEqual(rotated.vector.z, 1.0, places=6)
        self.assertAlmostEqual(rotated.vector.x, 0.0, places=6)
        self.assertAlmostEqual(rotated.vector.y, 0.0, places=6)

    def test_zero_axis_falls_back_to_input(self):
        cls = _pick_class()
        normal = _vec(0.3, 0.4, 0.5)
        zero_axis = _vec(0.0, 0.0, 0.0)
        rotated = cls._rotate_vector_stamped(normal, zero_axis, math.radians(45))
        self.assertAlmostEqual(rotated.vector.x, 0.3, places=6)
        self.assertAlmostEqual(rotated.vector.y, 0.4, places=6)
        self.assertAlmostEqual(rotated.vector.z, 0.5, places=6)


if __name__ == '__main__':
    unittest.main()
