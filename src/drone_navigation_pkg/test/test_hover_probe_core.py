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
    def test_planner_map_gate_requires_true_token_and_fresh_state(self):
        self.assertTrue(
            CORE.planner_map_ready(
                "WAITING_FOR_GOAL map_ready=true map_age=0.02 tf_age=0.02",
                0.2,
                0.6,
            )
        )
        self.assertFalse(
            CORE.planner_map_ready("WAITING map_ready=false", 0.2, 0.6)
        )
        self.assertFalse(CORE.planner_map_ready("", 0.2, 0.6))
        self.assertFalse(
            CORE.planner_map_ready("ACTIVE map_ready=true", 0.61, 0.6)
        )

    def test_prearm_pose_gate_requires_calibrated_spawn_speed_and_tilt(self):
        limits = CORE.PrearmPoseLimits(
            expected_position=(4.55, -0.38, 1.13),
            position_tolerance_m=0.02,
            max_speed_mps=0.05,
            max_tilt_deg=3.0,
        )
        official_spawn = CORE.PrearmPoseSample(
            position=(4.5513, -0.3819, 1.1299),
            speed_mps=0.0144,
            roll_deg=0.02,
            pitch_deg=-0.04,
        )
        floor_pose = CORE.PrearmPoseSample(
            position=(5.1876, -0.3052, 0.2630),
            speed_mps=0.0,
            roll_deg=0.0,
            pitch_deg=0.0,
        )

        self.assertTrue(CORE.prearm_pose_allowed(official_spawn, limits))
        self.assertFalse(CORE.prearm_pose_allowed(floor_pose, limits))
        self.assertFalse(
            CORE.prearm_pose_allowed(
                CORE.PrearmPoseSample(
                    position=official_spawn.position,
                    speed_mps=0.051,
                    roll_deg=0.0,
                    pitch_deg=0.0,
                ),
                limits,
            )
        )
        self.assertFalse(
            CORE.prearm_pose_allowed(
                CORE.PrearmPoseSample(
                    position=official_spawn.position,
                    speed_mps=0.0,
                    roll_deg=3.01,
                    pitch_deg=0.0,
                ),
                limits,
            )
        )

    def test_static_support_can_replace_missed_discrete_land_sample(self):
        self.assertTrue(CORE.ground_observation_usable(True, False))
        self.assertTrue(CORE.ground_observation_usable(False, True))
        self.assertFalse(CORE.ground_observation_usable(False, False))

    def test_takeoff_goal_requires_target_tolerance_and_support_clearance(self):
        self.assertFalse(CORE.takeoff_goal_reached(0.01, 1.10, 1.00, 1.25))
        self.assertTrue(CORE.takeoff_goal_reached(0.01, 1.21, 1.00, 1.25))
        self.assertFalse(CORE.takeoff_goal_reached(0.16, 1.25, 1.00, 1.25))

    def test_fixed_step_requires_position_and_speed_settling(self):
        self.assertTrue(CORE.fixed_step_reached(0.04, 0.03, 0.04))
        self.assertFalse(CORE.fixed_step_reached(0.11, 0.03, 0.04))
        self.assertFalse(CORE.fixed_step_reached(0.04, 0.06, 0.04))
        self.assertFalse(CORE.fixed_step_reached(0.04, 0.03, 0.051))

    def test_fixed_step_envelope_rejects_drift_drop_speed_and_tilt(self):
        self.assertTrue(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.201, 0.02, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.101, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.51, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.20, 20.1))

    def test_landing_region_is_valid_and_contains_only_safe_touchdown_area(self):
        region = CORE.LandingRegion(
            center_xy=(4.55, -0.38),
            half_extents_xy=(0.18, 0.18),
        )
        self.assertTrue(region.valid())
        self.assertTrue(region.contains((4.55, -0.38)))
        self.assertTrue(region.contains((4.73, -0.20)))
        self.assertFalse(region.contains((4.731, -0.20)))
        self.assertFalse(
            CORE.LandingRegion((0.0, 0.0), (0.0, 0.1)).valid()
        )
        self.assertFalse(CORE.verified_landing_region_available(region, False))
        self.assertTrue(CORE.verified_landing_region_available(region, True))

    def test_actuator_saturation_requires_finite_outputs_below_limit(self):
        self.assertFalse(CORE.actuator_outputs_saturated([0.2, 0.4, 0.6, 0.8]))
        self.assertTrue(CORE.actuator_outputs_saturated([0.2, 0.4, 0.95, 0.8]))
        self.assertFalse(
            CORE.actuator_outputs_saturated([float('nan'), 0.2, 0.3, 0.4])
        )
        self.assertTrue(
            CORE.actuator_outputs_saturated([float('nan')] * 4)
        )

    def test_hold_acceptance_rejects_error_speed_and_saturation(self):
        self.assertTrue(CORE.hold_acceptance_passes(0.10, 0.05, 0.05, False))
        self.assertFalse(CORE.hold_acceptance_passes(0.101, 0.05, 0.05, False))
        self.assertFalse(CORE.hold_acceptance_passes(0.10, 0.051, 0.05, False))
        self.assertFalse(CORE.hold_acceptance_passes(0.10, 0.05, 0.051, False))
        self.assertFalse(CORE.hold_acceptance_passes(0.10, 0.05, 0.05, True))
        self.assertFalse(
            CORE.hold_acceptance_passes(0.10, 0.05, 0.05, False, False)
        )

    def test_startup_reset_requires_disabled_loiter_disarmed_and_landed(self):
        self.assertTrue(CORE.startup_reset_ready("DISABLED", True, False, True))
        self.assertFalse(
            CORE.startup_reset_ready("LAND_LATCHED", True, False, True)
        )
        self.assertFalse(CORE.startup_reset_ready("DISABLED", False, False, True))
        self.assertFalse(CORE.startup_reset_ready("DISABLED", True, True, True))
        self.assertFalse(CORE.startup_reset_ready("DISABLED", True, False, False))

    def test_runtime_liveness_uses_continuous_topics_not_discrete_px4_state(self):
        self.assertIn("actuator_motors", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertIn("px4_sensor", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertIn("px4_odometry", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("px4_status", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("land_status", CORE.CONTINUOUS_FLIGHT_TOPICS)

    def test_clock_uses_independent_stale_timeout(self):
        ages = {
            "actuator_motors": 0.1,
            "clock": 2.0,
            "raw_pose": 0.1,
            "raw_twist": 0.1,
            "pointcloud": 0.1,
            "px4_sensor": 0.1,
            "px4_odometry": 0.1,
            "nav_odometry": 0.1,
            "planner_state": 0.1,
        }
        self.assertEqual(
            CORE.stale_flight_topics(ages, 1.5, 5.0),
            [],
        )
        ages["clock"] = 5.01
        self.assertEqual(
            CORE.stale_flight_topics(ages, 1.5, 5.0),
            ["clock"],
        )
        ages["clock"] = 0.1
        ages["px4_sensor"] = 1.51
        self.assertEqual(
            CORE.stale_flight_topics(ages, 1.5, 5.0),
            ["px4_sensor"],
        )

    def test_command_ack_requires_command_and_zero_result(self):
        self.assertTrue(CORE.successful_command_ack("command=176 result=0"))
        self.assertFalse(CORE.successful_command_ack("command=176 result=1"))
        self.assertFalse(CORE.successful_command_ack("result=0"))

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
        state = CORE.update_executor_lifecycle(
            "", "LIFECYCLE COMPLETE fixed_setpoint_enabled=true"
        )
        self.assertEqual(state, "COMPLETE")
        state = CORE.update_executor_lifecycle(state, "GROUND_DISARM_COMMAND_SENT")
        self.assertEqual(state, "COMPLETE")
        state = CORE.update_executor_lifecycle(state, "LIFECYCLE DISABLED")
        self.assertEqual(state, "DISABLED")
        state = CORE.update_executor_lifecycle(state, "RESET_LOITER_COMMAND_SENT")
        self.assertEqual(state, "DISABLED")


if __name__ == "__main__":
    unittest.main()
