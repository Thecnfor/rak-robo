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

    def test_fixed_diagnostic_can_ignore_only_planner_dependent_streams(self):
        ages = {
            name: 0.1 for name in CORE.CONTINUOUS_FLIGHT_TOPICS
        }
        ages["pointcloud"] = 2.0
        ages["planner_state"] = 2.0
        self.assertEqual(
            CORE.stale_flight_topics(
                ages,
                telemetry_timeout=1.5,
                clock_timeout=5.0,
                ignored_topics=("pointcloud", "planner_state"),
            ),
            [],
        )
        ages["nav_odometry"] = 2.0
        self.assertEqual(
            CORE.stale_flight_topics(
                ages,
                telemetry_timeout=1.5,
                clock_timeout=5.0,
                ignored_topics=("pointcloud", "planner_state"),
            ),
            ["nav_odometry"],
        )
        with self.assertRaises(ValueError):
            CORE.stale_flight_topics(
                ages,
                telemetry_timeout=1.5,
                clock_timeout=5.0,
                ignored_topics=("nav_odometry",),
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

    def test_default_prearm_gate_rejects_touchdown_at_guide_clearance_edge(self):
        limits = CORE.PrearmPoseLimits(
            expected_position=(4.55, -0.38, 1.13)
        )
        centered_after_door_close = CORE.PrearmPoseSample(
            position=(4.5539, -0.38, 1.13),
            speed_mps=0.0,
            roll_deg=0.0,
            pitch_deg=0.0,
        )
        guide_corner_touchdown = CORE.PrearmPoseSample(
            position=(4.559946, -0.365165, 1.13),
            speed_mps=0.0,
            roll_deg=0.0,
            pitch_deg=0.0,
        )
        self.assertTrue(
            CORE.prearm_pose_allowed(centered_after_door_close, limits)
        )
        self.assertFalse(CORE.prearm_pose_allowed(guide_corner_touchdown, limits))

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
        self.assertFalse(
            CORE.fixed_step_reached(
                0.04,
                0.03,
                0.041,
                max_speed_mps=0.04,
            )
        )

    def test_each_fixed_step_gets_a_fresh_timeout_origin(self):
        self.assertEqual(
            CORE.advance_fixed_step(0, 2, 12.5, 9_000_000_000),
            (1, False, 12.5, 9_000_000_000),
        )
        self.assertEqual(
            CORE.advance_fixed_step(1, 2, 14.0, 10_000_000_000),
            (2, True, None, None),
        )

    def test_high_altitude_fixed_probe_requires_explicit_bounded_authority(self):
        sequence = (0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.67)
        self.assertFalse(
            CORE.fixed_clearance_sequence_valid(
                sequence,
                hold_altitude_m=1.80,
                home_altitude_m=1.13,
                authorized_max_clearance_m=0.30,
            )
        )
        self.assertTrue(
            CORE.fixed_clearance_sequence_valid(
                sequence,
                hold_altitude_m=1.80,
                home_altitude_m=1.13,
                authorized_max_clearance_m=0.70,
            )
        )
        self.assertFalse(
            CORE.fixed_clearance_sequence_valid(
                sequence + (0.71,),
                hold_altitude_m=1.84,
                home_altitude_m=1.13,
                authorized_max_clearance_m=0.71,
            )
        )

    def test_fixed_step_envelope_rejects_drift_drop_speed_and_tilt(self):
        self.assertTrue(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.201, 0.02, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.101, 0.20, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.51, 8.0))
        self.assertFalse(CORE.fixed_step_envelope_safe(0.10, 0.02, 0.20, 20.1))

    def test_guided_horizontal_limit_stays_inside_physical_clearance(self):
        self.assertEqual(
            CORE.fixed_step_horizontal_limit(False, 0.012, 0.20),
            0.012,
        )
        self.assertEqual(
            CORE.fixed_step_horizontal_limit(True, 0.012, 0.20),
            0.20,
        )
        with self.assertRaises(ValueError):
            CORE.fixed_step_horizontal_limit(False, 0.015, 0.015)

    def test_full_horizontal_control_requires_explicit_executor_handoff(self):
        self.assertFalse(
            CORE.full_horizontal_control_available(
                "LIFECYCLE ACTIVE fixed_vertical_active=true"
            )
        )
        self.assertTrue(
            CORE.full_horizontal_control_available(
                "FIXED_CONTROL vertical_only=false fixed_vertical_active=false"
            )
        )
        self.assertFalse(CORE.full_horizontal_control_available("LIFECYCLE ACTIVE"))

    def test_full_horizontal_control_handoff_latches_until_round_reset(self):
        latched = CORE.update_full_horizontal_control_latch(
            False,
            "FIXED_CONTROL vertical_only=false fixed_vertical_active=false",
        )
        self.assertTrue(latched)
        self.assertTrue(
            CORE.update_full_horizontal_control_latch(
                latched,
                "LIFECYCLE ACTIVE fixed_vertical_active=true",
            )
        )
        self.assertFalse(
            CORE.update_full_horizontal_control_latch(
                latched,
                "LIFECYCLE ACTIVE fixed_vertical_active=true",
                reset=True,
            )
        )

    def test_abort_return_requires_positive_airborne_clearance(self):
        args = (True, False, True, True, True, True)
        self.assertFalse(
            CORE.abort_return_allowed(
                *args, 1.025, 1.0, full_horizontal_control=True
            )
        )
        self.assertTrue(
            CORE.abort_return_allowed(
                *args, 1.03, 1.0, full_horizontal_control=True
            )
        )
        self.assertFalse(
            CORE.abort_return_allowed(
                *args, 1.20, 1.0, full_horizontal_control=False
            )
        )
        self.assertFalse(
            CORE.abort_return_allowed(
                True, True, True, True, True, True, 1.2, 1.0
            )
        )
        self.assertFalse(
            CORE.abort_return_allowed(
                True, False, False, True, True, True, 1.2, 1.0
            )
        )

    def test_target_translation_preserves_displacement_between_origins(self):
        raw_home = (4.5599, -0.3799, 1.1300)
        nav_home = (4.5536, -0.3768, 1.1135)
        raw_target = (4.5510, -0.3800, 1.2300)
        nav_target = CORE.translate_target_between_origins(
            raw_target, raw_home, nav_home
        )
        self.assertAlmostEqual(nav_target[0], 4.5447, places=4)
        self.assertAlmostEqual(nav_target[1], -0.3769, places=4)
        self.assertAlmostEqual(nav_target[2], 1.2135, places=4)
        round_trip = CORE.translate_target_between_origins(
            nav_target, nav_home, raw_home
        )
        for actual, expected in zip(round_trip, raw_target):
            self.assertAlmostEqual(actual, expected)
        with self.assertRaises(ValueError):
            CORE.translate_target_between_origins(
                (0.0, 0.0), nav_home, raw_home
            )

    def test_live_raw_error_correction_rejects_estimator_drift_on_return(self):
        raw_target = (4.55, -0.38, 1.21)
        raw_position = (4.54, -0.355, 1.20)
        nav_position = (0.012, -0.007, -0.071)
        nav_target = CORE.live_raw_error_corrected_nav_target(
            raw_target,
            raw_position,
            nav_position,
        )
        for actual, expected in zip(nav_target, (0.022, -0.032, -0.061)):
            self.assertAlmostEqual(actual, expected)
        with self.assertRaises(ValueError):
            CORE.live_raw_error_corrected_nav_target(
                raw_target,
                raw_position[:2],
                nav_position,
            )

    def test_live_frame_offset_filter_rejects_asynchronous_centimeter_jumps(self):
        previous = (0.004, -0.003, -0.016)
        observed = (0.024, -0.023, -0.006)
        filtered = CORE.low_pass_frame_offset(
            previous,
            observed,
            elapsed_seconds=0.1,
            time_constant_seconds=2.0,
            maximum_rate_mps=0.02,
        )
        update = tuple(
            filtered[index] - previous[index] for index in range(3)
        )
        self.assertLessEqual(
            sum(value * value for value in update) ** 0.5,
            0.002 + 1e-12,
        )
        self.assertTrue(
            all(
                abs(filtered[index] - observed[index])
                < abs(previous[index] - observed[index])
                for index in range(3)
            )
        )
        self.assertEqual(
            CORE.low_pass_frame_offset(
                filtered,
                observed,
                elapsed_seconds=0.0,
                time_constant_seconds=2.0,
                maximum_rate_mps=0.02,
            ),
            filtered,
        )
        with self.assertRaises(ValueError):
            CORE.low_pass_frame_offset(
                previous,
                observed,
                elapsed_seconds=0.1,
                time_constant_seconds=0.0,
                maximum_rate_mps=0.02,
            )

    def test_landing_frame_offset_is_frozen_on_first_return_sample(self):
        home_raw = (4.55, -0.38, 1.13)
        home_nav = (4.544, -0.377, 1.114)
        live_at_return = (0.004, 0.005, -0.015)
        frozen = CORE.freeze_landing_frame_offset(
            None,
            live_at_return,
            home_raw,
            home_nav,
        )
        self.assertEqual(frozen, live_at_return)
        self.assertEqual(
            CORE.freeze_landing_frame_offset(
                frozen,
                (-0.007, 0.014, -0.021),
                home_raw,
                home_nav,
            ),
            frozen,
        )
        self.assertEqual(
            CORE.freeze_landing_frame_offset(
                None,
                None,
                home_raw,
                home_nav,
            ),
            (
                home_nav[0] - home_raw[0],
                home_nav[1] - home_raw[1],
                home_nav[2] - home_raw[2],
            ),
        )
        with self.assertRaises(ValueError):
            CORE.freeze_landing_frame_offset(
                None,
                (float("nan"), 0.0, 0.0),
                home_raw,
                home_nav,
            )

    def test_landing_frame_capture_uses_hysteresis_at_the_truth_gate(self):
        self.assertEqual(
            CORE.landing_frame_capture_action(
                frozen=False,
                horizontal_error_m=0.020,
                speed_mps=0.01,
                capture_radius_m=0.008,
                release_radius_m=0.012,
            ),
            "live",
        )
        self.assertEqual(
            CORE.landing_frame_capture_action(
                frozen=False,
                horizontal_error_m=0.007,
                speed_mps=0.009,
                capture_radius_m=0.008,
                release_radius_m=0.012,
                maximum_capture_speed_mps=0.01,
            ),
            "capture",
        )
        self.assertEqual(
            CORE.landing_frame_capture_action(
                frozen=True,
                horizontal_error_m=0.010,
                speed_mps=0.01,
                capture_radius_m=0.008,
                release_radius_m=0.012,
            ),
            "hold",
        )
        self.assertEqual(
            CORE.landing_frame_capture_action(
                frozen=True,
                horizontal_error_m=0.013,
                speed_mps=0.01,
                capture_radius_m=0.008,
                release_radius_m=0.012,
            ),
            "release",
        )
        self.assertEqual(
            CORE.landing_frame_capture_action(
                frozen=False,
                horizontal_error_m=0.007,
                speed_mps=0.011,
                capture_radius_m=0.008,
                release_radius_m=0.012,
                maximum_capture_speed_mps=0.01,
            ),
            "live",
        )
        with self.assertRaises(ValueError):
            CORE.landing_frame_capture_action(
                frozen=False,
                horizontal_error_m=0.0,
                speed_mps=0.0,
                capture_radius_m=0.012,
                release_radius_m=0.008,
            )

    def test_landing_target_freezes_without_a_capture_step(self):
        raw_target = (4.55, -0.38, 1.21)
        raw_position = (4.547, -0.374, 1.205)
        nav_position = (0.010, -0.004, -0.077)
        live_target = CORE.landing_nav_target(
            raw_target,
            raw_position,
            nav_position,
            frozen_offset=None,
            horizontal_gain=2.0,
        )
        captured_offset = tuple(
            live_target[index] - raw_target[index]
            for index in range(3)
        )
        captured_target = CORE.landing_nav_target(
            raw_target,
            raw_position,
            nav_position,
            frozen_offset=captured_offset,
            horizontal_gain=2.0,
        )
        changed_inputs_target = CORE.landing_nav_target(
            raw_target,
            (4.560, -0.390, 1.210),
            (0.025, -0.020, -0.072),
            frozen_offset=captured_offset,
            horizontal_gain=2.0,
        )
        for expected, captured, changed in zip(
            live_target,
            captured_target,
            changed_inputs_target,
        ):
            self.assertAlmostEqual(captured, expected)
            self.assertAlmostEqual(changed, expected)
        self.assertAlmostEqual(
            live_target[0],
            nav_position[0] + 2.0 * (raw_target[0] - raw_position[0]),
        )

    def test_yaw_rate_envelope_rejects_nonfinite_and_excessive_rotation(self):
        self.assertTrue(CORE.yaw_rate_envelope_safe(0.0, 1.0))
        self.assertTrue(CORE.yaw_rate_envelope_safe(-1.0, 1.0))
        self.assertFalse(CORE.yaw_rate_envelope_safe(1.001, 1.0))
        self.assertFalse(CORE.yaw_rate_envelope_safe(float("nan"), 1.0))
        self.assertFalse(CORE.yaw_rate_envelope_safe(0.1, 0.0))

    def test_force_disarm_fallback_requires_stationary_flipped_crash(self):
        args = dict(
            enabled=True,
            armed=True,
            auto_land=True,
            executor_lifecycle="LAND_LATCHED",
            current_z=0.018,
            spawn_z=1.13,
            speed_mps=0.001,
            roll_deg=179.0,
            pitch_deg=2.0,
            stationary_duration_s=1.0,
        )
        self.assertTrue(CORE.crashed_airframe_force_disarm_ready(**args))
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "enabled": False}
            )
        )
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "auto_land": False}
            )
        )
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "current_z": 1.0}
            )
        )
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "speed_mps": 0.051}
            )
        )
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "roll_deg": 10.0, "pitch_deg": 10.0}
            )
        )
        self.assertFalse(
            CORE.crashed_airframe_force_disarm_ready(
                **{**args, "stationary_duration_s": 0.99}
            )
        )

    def test_prearm_attitude_agreement_rejects_unconverged_px4_tilt(self):
        self.assertTrue(
            CORE.prearm_attitude_agreement_allowed(0.0, 0.0, 0.4, -0.4)
        )
        self.assertFalse(
            CORE.prearm_attitude_agreement_allowed(0.0, 0.0, 0.6, 0.0)
        )
        self.assertTrue(
            CORE.prearm_attitude_agreement_allowed(179.8, 0.0, -179.8, 0.0)
        )
        self.assertFalse(
            CORE.prearm_attitude_agreement_allowed(
                0.0, 0.0, float("nan"), 0.0
            )
        )

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
        self.assertEqual(
            CORE.landing_approach_target(region, 1.13, 0.10),
            (4.55, -0.38, 1.23),
        )
        self.assertEqual(
            CORE.cradle_touchdown_target(region, 1.13),
            (4.55, -0.38, 1.13),
        )
        self.assertEqual(
            CORE.cradle_approach_target(region, 1.13, 0.08),
            (4.55, -0.38, 1.21),
        )
        with self.assertRaises(ValueError):
            CORE.cradle_approach_target(region, 1.13, 0.0)
        self.assertTrue(
            CORE.guided_touchdown_reached(
                0.008,
                0.015,
                0.05,
                guide_radius_m=0.008,
            )
        )
        self.assertFalse(
            CORE.guided_touchdown_reached(
                0.0081,
                0.015,
                0.05,
                guide_radius_m=0.008,
            )
        )
        self.assertFalse(
            CORE.guided_touchdown_reached(
                0.003,
                0.015,
                0.05,
                guide_radius_m=0.0081,
            )
        )

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
        self.assertNotIn("actuator_motors", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertIn("px4_sensor", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertIn("px4_odometry", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("px4_status", CORE.CONTINUOUS_FLIGHT_TOPICS)
        self.assertNotIn("land_status", CORE.CONTINUOUS_FLIGHT_TOPICS)

    def test_clock_uses_independent_stale_timeout(self):
        ages = {
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
