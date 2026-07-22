#!/usr/bin/env python3

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).parents[1] / "scripts" / "hover_probe_core.py"
SPEC = importlib.util.spec_from_file_location("hover_probe_core", MODULE_PATH)
CORE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CORE)


class Endpoint:
    def __init__(self, name, namespace="/"):
        self.node_name = name
        self.node_namespace = namespace


class HoverProbeCoreTest(unittest.TestCase):
    def test_static_support_can_replace_missed_discrete_land_sample(self):
        self.assertTrue(CORE.ground_observation_usable(True, False))
        self.assertTrue(CORE.ground_observation_usable(False, True))
        self.assertFalse(CORE.ground_observation_usable(False, False))

    def test_takeoff_goal_requires_target_tolerance_and_support_clearance(self):
        self.assertFalse(CORE.takeoff_goal_reached(0.01, 1.10, 1.00, 1.25))
        self.assertTrue(CORE.takeoff_goal_reached(0.01, 1.21, 1.00, 1.25))
        self.assertFalse(CORE.takeoff_goal_reached(0.16, 1.25, 1.00, 1.25))

    def test_startup_reset_requires_disabled_loiter_disarmed_and_landed(self):
        self.assertTrue(CORE.startup_reset_ready("DISABLED", True, False, True))
        self.assertFalse(
            CORE.startup_reset_ready("LAND_LATCHED", True, False, True)
        )
        self.assertFalse(CORE.startup_reset_ready("DISABLED", False, False, True))
        self.assertFalse(CORE.startup_reset_ready("DISABLED", True, True, True))
        self.assertFalse(CORE.startup_reset_ready("DISABLED", True, False, False))

    def test_runtime_liveness_uses_continuous_topics_not_discrete_px4_state(self):
        self.assertIn("px4_sensor", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertIn("px4_odometry", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("px4_status", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("land_status", CORE.CONTINUOUS_FLIGHT_TOPICS)

    def test_parses_cargo_boolean_tokens(self):
        text = "payload_locked=True prearm_support=false"
        self.assertTrue(CORE.parse_bool_token(text, "payload_locked"))
        self.assertFalse(CORE.parse_bool_token(text, "prearm_support"))
        self.assertIsNone(CORE.parse_bool_token(text, "missing"))

    def test_stability_requires_duration_drift_speed_and_tilt(self):
        window = CORE.StabilityWindow(4.0)
        window.add(0.0, (1.0, 2.0, 3.0), 0.01, 0.0, 0.0)
        window.add(3.1, (1.01, 2.0, 3.0), 0.04, 2.0, -1.0)
        self.assertTrue(window.report().passes())
        window.add(3.2, (1.04, 2.0, 3.0), 0.04, 2.0, -1.0)
        self.assertFalse(window.report().passes())

    def test_writer_gate_rejects_missing_or_duplicate_publishers(self):
        expected = Endpoint("trajectory_executor")
        duplicate = Endpoint("bad_writer", "/debug")
        self.assertTrue(CORE.sole_writer_is([expected], "/trajectory_executor"))
        self.assertFalse(CORE.sole_writer_is([], "/trajectory_executor"))
        self.assertFalse(
            CORE.sole_writer_is([expected, duplicate], "/trajectory_executor")
        )

    def test_phase_elapsed_prefers_monotonic_simulation_time(self):
        self.assertAlmostEqual(
            CORE.phase_elapsed_seconds(10.0, 40.0, 2_000_000_000, 3_500_000_000),
            1.5,
        )
        self.assertAlmostEqual(
            CORE.phase_elapsed_seconds(10.0, 40.0, None, None),
            30.0,
        )
        self.assertAlmostEqual(
            CORE.phase_elapsed_seconds(10.0, 40.0, 3_500_000_000, 2_000_000_000),
            30.0,
        )

    def test_executor_lifecycle_ignores_interleaved_event_messages(self):
        state = CORE.update_executor_lifecycle("", "LIFECYCLE COMPLETE")
        self.assertEqual(state, "COMPLETE")
        state = CORE.update_executor_lifecycle(state, "GROUND_DISARM_COMMAND_SENT")
        self.assertEqual(state, "COMPLETE")
        state = CORE.update_executor_lifecycle(state, "LIFECYCLE DISABLED")
        self.assertEqual(state, "DISABLED")
        state = CORE.update_executor_lifecycle(state, "RESET_LOITER_COMMAND_SENT")
        self.assertEqual(state, "DISABLED")


if __name__ == "__main__":
    unittest.main()
