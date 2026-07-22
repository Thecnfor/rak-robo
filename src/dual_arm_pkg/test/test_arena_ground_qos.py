"""Contract test for the air-ground mission handoff QoS."""

import unittest

try:
    import rclpy  # noqa: F401

    _HAVE_RCLPY = True
except ImportError:
    _HAVE_RCLPY = False


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class ArenaGroundStateQosTest(unittest.TestCase):
    def test_ground_state_publisher_is_transient_local(self):
        from rclpy.qos import DurabilityPolicy

        from dual_arm_pkg.dual_arm_pick_place_node import _ARENA_STATE_QOS

        self.assertEqual(
            _ARENA_STATE_QOS.durability,
            DurabilityPolicy.TRANSIENT_LOCAL,
        )


if __name__ == '__main__':
    unittest.main()
