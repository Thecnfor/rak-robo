"""Unit tests for dual_arm_pkg state machines and gripper position mapping.

The tests do NOT spin ROS 2 nodes; they target the pure-Python helper
behaviour so the package can be exercised without a DDS discovery
context.
"""
import unittest


# ``rclpy`` is only available when ROS 2 is on PYTHONPATH (i.e. inside an
# ``colcon test`` invocation that pulls /opt/ros/jazzy into
# AMENT_PREFIX_PATH). When invoked from a system shell the import below
# still raises an ImportError; test classes that need it guard each
# method so the runner reports a skip rather than a 0-tests error.
try:
    import rclpy  # noqa: F401

    _HAVE_RCLPY = True
except ImportError:
    _HAVE_RCLPY = False


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class GripperPositionTest(unittest.TestCase):
    def test_open_returns_open_position(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position('open'), 0.04)

    def test_close_returns_closed_position(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position('close'), 0.0)

    def test_stop_equals_close(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position('stop'), 0.0)

    def test_unknown_command_defaults_open(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position('xyz'), 0.04)

    def test_empty_string_defaults_open(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position(''), 0.04)

    def test_whitespace_stripping(self):
        from dual_arm_pkg.joint_order import gripper_position
        self.assertAlmostEqual(gripper_position('  close  '), 0.0)


class DualActionSurfaceTest(unittest.TestCase):
    def test_class_exposes_dual_command_class(self):
        # Both GripperCommand + DualGripperCommand must be importable
        # through the action server module when interfaces have been built.
        try:
            from grasp_demo_interfaces.action import (
                DualGripperCommand,
                GripperCommand,
            )
            self.assertTrue(hasattr(GripperCommand, 'Goal'))
            self.assertTrue(hasattr(DualGripperCommand, 'Goal'))
        except ImportError:
            self.skipTest('grasp_demo_interfaces not built')


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class CollisionKeepoutTest(unittest.TestCase):
    """The heuristic collision check publishes a Bool when the planned
    arm targets violate the X1 chest keepout. We assert the constant
    values without spinning ROS 2."""

    def test_keepout_constants_within_x1_chest(self):
        from dual_arm_pkg.dual_arm_pick_place_node import (
            _COLLISION_KEEPOUT_X,
            _COLLISION_KEEPOUT_Y,
        )
        # Keepouts must lie inside the X1 chest envelope and stay
        # positive so the heuristic collision check never flags a
        # safe configuration by mistake.
        self.assertLessEqual(_COLLISION_KEEPOUT_X, 0.12)
        self.assertLessEqual(_COLLISION_KEEPOUT_Y, 0.12)
        self.assertGreater(_COLLISION_KEEPOUT_X, 0.05)
        self.assertGreater(_COLLISION_KEEPOUT_Y, 0.02)


@unittest.skipUnless(_HAVE_RCLPY, 'rclpy not on PYTHONPATH')
class PickPlacePhaseEnumTest(unittest.TestCase):
    def test_phases_are_str_values(self):
        from dual_arm_pkg.dual_arm_pick_place_node import _Phase
        self.assertEqual(_Phase.IDLE.value, 'IDLE')
        self.assertEqual(_Phase.PARALLEL_DETECT.value, 'PARALLEL_DETECT')
        self.assertEqual(_Phase.DUAL_GRASP.value, 'DUAL_GRASP')
        self.assertEqual(_Phase.LIFTING.value, 'LIFTING')
        self.assertEqual(_Phase.PLACING.value, 'PLACING')
        self.assertEqual(_Phase.COMPLETE.value, 'COMPLETE')
        self.assertEqual(_Phase.FAILED.value, 'FAILED')

    def test_phase_transitions_stay_string_values(self):
        from dual_arm_pkg.dual_arm_pick_place_node import _Phase, _State
        state = _State()
        self.assertEqual(state.phase, _Phase.IDLE)
        state.phase = _Phase.PARALLEL_DETECT
        self.assertEqual(state.phase.value, 'PARALLEL_DETECT')
        state.phase = _Phase.COMPLETE
        self.assertEqual(state.phase.value, 'COMPLETE')


if __name__ == '__main__':
    unittest.main()
