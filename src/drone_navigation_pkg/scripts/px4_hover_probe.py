#!/usr/bin/env python3
"""Safety-gated PX4 takeoff, hover, and landing acceptance probe.

This node never writes /fmu/in/*. It drives the same operator intent topics used
by the competition supervisor and refuses to arm unless every live-data gate is
healthy and trajectory_executor is the sole PX4 input writer.
"""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    ActuatorMotors,
    SensorCombined,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from hover_probe_core import (
    CONTINUOUS_FLIGHT_TOPICS,
    LandingRegion,
    PrearmPoseLimits,
    PrearmPoseSample,
    RateWindow,
    StabilityWindow,
    abort_return_allowed,
    actuator_outputs_saturated,
    advance_fixed_step,
    cradle_touchdown_target,
    crashed_airframe_force_disarm_ready,
    fixed_step_envelope_safe,
    fixed_step_horizontal_limit,
    fixed_step_reached,
    full_horizontal_control_available,
    ground_observation_usable,
    hold_acceptance_passes,
    landing_approach_target,
    normalized_node_names,
    parse_bool_token,
    phase_elapsed_seconds,
    planner_map_ready,
    prearm_attitude_agreement_allowed,
    prearm_pose_allowed,
    quaternion_roll_pitch_degrees,
    sole_writer_is,
    stale_flight_topics,
    startup_reset_ready,
    successful_command_ack,
    takeoff_goal_reached,
    translate_target_between_origins,
    update_executor_lifecycle,
    verified_landing_region_available,
    yaw_rate_envelope_safe,
)


PX4_INPUT_TOPICS = (
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/vehicle_command",
)


class HoverProbe(Node):
    def __init__(self):
        super().__init__("px4_hover_probe")
        self.rounds = int(self.declare_parameter("rounds", 3).value)
        self.hover_seconds = float(self.declare_parameter("hover_seconds", 10.0).value)
        self.goal_altitude = float(self.declare_parameter("goal_altitude", 1.8).value)
        self.land_wall_timeout = float(
            self.declare_parameter("land_wall_timeout", 180.0).value
        )
        self.flight_data_timeout = float(
            self.declare_parameter("flight_data_timeout", 1.5).value
        )
        self.flight_clock_timeout = float(
            self.declare_parameter("flight_clock_timeout", 5.0).value
        )
        self.planner_map_timeout = float(
            self.declare_parameter("planner_map_timeout", 0.6).value
        )
        self.preflight_only = bool(self.declare_parameter("preflight_only", True).value)
        self.fixed_setpoint_diagnostic = bool(
            self.declare_parameter("fixed_setpoint_diagnostic", False).value
        )
        self.fixed_step_clearances = tuple(
            float(value)
            for value in self.declare_parameter(
                "fixed_step_clearances", [0.10, 0.20]
            ).value
        )
        self.fixed_hold_altitude = float(
            self.declare_parameter("fixed_hold_altitude", 1.30).value
        )
        self.fixed_step_settle_seconds = float(
            self.declare_parameter("fixed_step_settle_seconds", 2.0).value
        )
        self.fixed_step_timeout = float(
            self.declare_parameter("fixed_step_timeout", 15.0).value
        )
        self.fixed_target_horizontal_tolerance = float(
            self.declare_parameter("fixed_target_horizontal_tolerance", 0.10).value
        )
        self.fixed_target_altitude_tolerance = float(
            self.declare_parameter("fixed_target_altitude_tolerance", 0.05).value
        )
        self.fixed_step_max_speed_mps = float(
            self.declare_parameter("fixed_step_max_speed_mps", 0.30).value
        )
        self.fixed_step_max_tilt_deg = float(
            self.declare_parameter("fixed_step_max_tilt_deg", 15.0).value
        )
        self.fixed_guided_horizontal_limit_m = float(
            self.declare_parameter(
                "fixed_guided_horizontal_limit_m", 0.012
            ).value
        )
        self.fixed_full_xy_horizontal_limit_m = float(
            self.declare_parameter(
                "fixed_full_xy_horizontal_limit_m", 0.20
            ).value
        )
        self.cradle_touchdown = bool(
            self.declare_parameter("cradle_touchdown", False).value
        )
        landing_half_extents = tuple(
            float(value)
            for value in self.declare_parameter(
                "landing_half_extents", [0.18, 0.18]
            ).value
        )
        landing_center = tuple(
            float(value)
            for value in self.declare_parameter(
                "landing_region_center", [4.55, -0.38]
            ).value
        )
        self.landing_region_verified = bool(
            self.declare_parameter("landing_region_verified", False).value
        )
        self.actuator_saturation_threshold = float(
            self.declare_parameter("actuator_saturation_threshold", 0.95).value
        )
        self.actuator_acceptance_timeout = float(
            self.declare_parameter("actuator_acceptance_timeout", 0.30).value
        )
        self.require_live_actuator_feedback = bool(
            self.declare_parameter("require_live_actuator_feedback", False).value
        )
        self.max_yaw_rate_rad_s = float(
            self.declare_parameter("max_yaw_rate_rad_s", 0.5).value
        )
        self.enable_force_disarm_diagnostic = bool(
            self.declare_parameter("enable_force_disarm_diagnostic", False).value
        )
        self.force_disarm_crash_delay = float(
            self.declare_parameter("force_disarm_crash_delay", 1.0).value
        )
        actuator_motors_topic = str(
            self.declare_parameter(
                "actuator_motors_topic", "/fmu/out/actuator_motors"
            ).value
        )
        command_ack_topic = str(
            self.declare_parameter(
                "px4_command_ack_topic", "/drone/navigation/px4_command_ack"
            ).value
        )
        self.landing_return_timeout = float(
            self.declare_parameter("landing_return_timeout", 15.0).value
        )
        spawn = list(
            self.declare_parameter(
                "prearm_spawn_position", [4.55, -0.38, 1.13]
            ).value
        )
        if len(spawn) != 3:
            raise ValueError("prearm_spawn_position must contain [x, y, z]")
        self.prearm_limits = PrearmPoseLimits(
            expected_position=tuple(float(value) for value in spawn),
            position_tolerance_m=float(
                self.declare_parameter("prearm_position_tolerance", 0.015).value
            ),
            max_speed_mps=float(
                self.declare_parameter("prearm_max_speed", 0.05).value
            ),
            max_tilt_deg=float(
                self.declare_parameter("prearm_max_tilt_deg", 3.0).value
            ),
        )
        self.prearm_attitude_tolerance_deg = float(
            self.declare_parameter("prearm_attitude_tolerance_deg", 0.5).value
        )
        self.landing_region = LandingRegion(
            center_xy=landing_center,
            half_extents_xy=landing_half_extents,
        )
        self.output_path = Path(
            str(self.declare_parameter("output_path", "/tmp/drone_hover_probe.json").value)
        )
        fixed_step_horizontal_limit(
            False,
            self.fixed_guided_horizontal_limit_m,
            self.fixed_full_xy_horizontal_limit_m,
        )
        if (
            self.fixed_guided_horizontal_limit_m
            >= self.prearm_limits.position_tolerance_m
        ):
            raise ValueError(
                "guided horizontal limit must remain inside guide clearance"
            )
        if (
            self.rounds < 1
            or self.hover_seconds <= 0.0
            or self.land_wall_timeout <= 0.0
            or self.flight_data_timeout <= 0.0
            or self.flight_clock_timeout <= 0.0
            or self.planner_map_timeout <= 0.0
            or self.prearm_limits.position_tolerance_m < 0.0
            or self.prearm_limits.max_speed_mps < 0.0
            or self.prearm_limits.max_tilt_deg < 0.0
            or self.prearm_attitude_tolerance_deg < 0.0
            or not self.fixed_step_clearances
            or any(
                clearance <= 0.0 or clearance > 0.30
                for clearance in self.fixed_step_clearances
            )
            or any(
                later <= earlier
                for earlier, later in zip(
                    self.fixed_step_clearances,
                    self.fixed_step_clearances[1:],
                )
            )
            or self.fixed_hold_altitude <= self.prearm_limits.expected_position[2]
            or self.fixed_hold_altitude > self.prearm_limits.expected_position[2] + 0.30
            or self.fixed_step_settle_seconds <= 0.0
            or self.fixed_step_timeout <= 0.0
            or self.fixed_target_horizontal_tolerance <= 0.0
            or self.fixed_target_altitude_tolerance <= 0.0
            or self.fixed_step_max_speed_mps <= 0.0
            or self.fixed_step_max_tilt_deg <= 0.0
            or self.landing_return_timeout <= 0.0
            or not self.landing_region.valid()
            or not 0.0 < self.actuator_saturation_threshold <= 1.0
            or self.actuator_acceptance_timeout <= 0.0
            or self.max_yaw_rate_rad_s <= 0.0
            or self.force_disarm_crash_delay < 0.0
        ):
            raise ValueError("probe durations and timeouts must be positive")

        transient = QoSProfile(depth=10)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        transient.reliability = ReliabilityPolicy.RELIABLE
        self.goal_pub = self.create_publisher(
            PoseStamped, "/drone/navigation/operator_goal", transient
        )
        self.mode_pub = self.create_publisher(
            String, "/drone/navigation/operator_mode", transient
        )
        self.cargo_pub = self.create_publisher(String, "/cargo_bay/command", 10)

        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/drone0/state/pose", self._on_raw_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, "/drone0/state/twist", self._on_raw_twist, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/avoidance/lidar/pointcloud",
            lambda _: self._mark("pointcloud"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ActuatorMotors,
            actuator_motors_topic,
            self._on_actuator_motors,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            lambda _: self._mark("px4_sensor"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            lambda _: self._mark("px4_odometry"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._on_vehicle_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self._on_land_detected,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/drone/navigation/odometry",
            self._on_navigation_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/drone/navigation/executor_state", self._on_executor_state, transient
        )
        self.create_subscription(
            String, "/drone/navigation/planner_state", self._on_planner_state, transient
        )
        self.create_subscription(
            String, command_ack_topic, self._on_command_ack, 10
        )
        self.create_subscription(String, "/cargo_bay/status", self._on_cargo_status, 10)

        self.trackers = {
            name: RateWindow(5.0)
            for name in (
                "actuator_motors",
                "clock",
                "raw_pose",
                "raw_twist",
                "pointcloud",
                "px4_sensor",
                "px4_odometry",
                "px4_status",
                "land_status",
                "nav_odometry",
                "planner_state",
            )
        }
        self.stability = StabilityWindow(4.0)
        # RESET is an explicit terminal-latch protocol. Always establish a
        # known grounded baseline before evaluating preflight or arming.
        self.phase = "RESET_START"
        self.phase_started = time.monotonic()
        self.phase_started_sim_ns = None
        self.started = self.phase_started
        self.finished = False
        self.success = False
        self.failure_reason = ""
        self.events = []
        self.round_results = []
        self.current_round = 0
        self.last_periodic_command = 0.0
        self.last_gate_log = 0.0
        self.last_reset_log = 0.0
        self.clock_last_ns = None
        self.clock_monotonic = True
        self.raw_position = None
        self.raw_speed = 0.0
        self.raw_yaw_rate = 0.0
        self.max_yaw_rate_seen = 0.0
        self.raw_roll = 0.0
        self.raw_pitch = 0.0
        self.nav_position = None
        self.nav_roll = None
        self.nav_pitch = None
        self.home_nav = None
        self.home_raw = None
        self.goal_stable_since = None
        self.goal_stable_since_sim_ns = None
        self.vehicle_status = None
        self.land_status = None
        self.executor_state = ""
        self.executor_lifecycle = ""
        self.planner_state = ""
        self.side_door_closed = False
        self.bottom_door_closed = False
        self.payload_locked = None
        self.prearm_support = None
        self.hold_max_horizontal_error = 0.0
        self.hold_max_altitude_error = 0.0
        self.hold_max_speed = 0.0
        self.hold_max_actuator_output = 0.0
        self.hold_saturation_seen = False
        self.have_actuator_motors = False
        self.command_acks = []
        self.round_ack_start = 0
        self.abort_reset_since = None
        self.abort_landing_timeout_reported = False
        self.force_disarm_crash_since = None
        self.force_disarm_request_logged = False
        self.commanded_goal = None
        self.fixed_step_index = 0
        self.fixed_settle_since = None
        self.fixed_settle_since_sim_ns = None
        self.abort_return_altitude = None
        self.touchdown_position = None
        self.last_gates = {}
        self.timer = self.create_timer(0.1, self._tick)
        self._event(
            "probe_started",
            preflight_only=self.preflight_only,
            fixed_setpoint_diagnostic=self.fixed_setpoint_diagnostic,
        )

    @staticmethod
    def _now():
        return time.monotonic()

    def _mark(self, name):
        self.trackers[name].add(self._now())

    def _on_clock(self, message):
        current_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.clock_last_ns is not None and current_ns < self.clock_last_ns:
            self.clock_monotonic = False
        self.clock_last_ns = current_ns
        self._mark("clock")

    def _on_raw_twist(self, message):
        linear = message.twist.linear
        self.raw_speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
        self.raw_yaw_rate = message.twist.angular.z
        if math.isfinite(self.raw_yaw_rate):
            self.max_yaw_rate_seen = max(
                self.max_yaw_rate_seen, abs(self.raw_yaw_rate)
            )
        self._mark("raw_twist")

    def _on_raw_pose(self, message):
        position = message.pose.position
        orientation = message.pose.orientation
        self.raw_position = (position.x, position.y, position.z)
        self.raw_roll, self.raw_pitch = quaternion_roll_pitch_degrees(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        now = self._now()
        self.trackers["raw_pose"].add(now)
        self.stability.add(
            now,
            self.raw_position,
            self.raw_speed,
            self.raw_roll,
            self.raw_pitch,
        )

    def _on_vehicle_status(self, message):
        self.vehicle_status = message
        self._mark("px4_status")

    def _on_actuator_motors(self, message):
        outputs = [value for value in message.control if math.isfinite(value)]
        self.have_actuator_motors = bool(outputs)
        self._mark("actuator_motors")
        current_max = max((abs(value) for value in outputs), default=0.0)
        if self.phase in {"FIXED_HOLD", "HOLD"}:
            self.hold_max_actuator_output = max(
                self.hold_max_actuator_output, current_max
            )
            self.hold_saturation_seen = self.hold_saturation_seen or (
                actuator_outputs_saturated(
                    outputs, self.actuator_saturation_threshold
                )
            )

    def _on_command_ack(self, message):
        self.command_acks.append(message.data)
        self._event("px4_command_ack", value=message.data)

    def _on_land_detected(self, message):
        self.land_status = message
        self._mark("land_status")

    def _on_navigation_odometry(self, message):
        position = message.pose.pose.position
        self.nav_position = (position.x, position.y, position.z)
        orientation = message.pose.pose.orientation
        self.nav_roll, self.nav_pitch = quaternion_roll_pitch_degrees(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self._mark("nav_odometry")

    def _on_executor_state(self, message):
        self.executor_state = message.data
        self.executor_lifecycle = update_executor_lifecycle(
            self.executor_lifecycle, message.data
        )

    def _on_planner_state(self, message):
        self.planner_state = message.data
        self._mark("planner_state")

    def _on_cargo_status(self, message):
        text = message.data.lower()
        if "left_closed" in text or "side_closed" in text:
            self.side_door_closed = True
        if "left_opened" in text or "side_opened" in text:
            self.side_door_closed = False
        if "bottom_closed" in text:
            self.bottom_door_closed = True
        if "bottom_opened" in text:
            self.bottom_door_closed = False
        payload_locked = parse_bool_token(text, "payload_locked")
        support = parse_bool_token(text, "prearm_support")
        if payload_locked is not None:
            self.payload_locked = payload_locked
        if support is not None:
            self.prearm_support = support

    def _event(self, name, **fields):
        event = {"time": round(self._now() - self.started, 3), "event": name}
        event.update(fields)
        self.events.append(event)
        self.get_logger().info(json.dumps(event, ensure_ascii=False, sort_keys=True))

    def _publish_text(self, publisher, text):
        message = String()
        message.data = text
        publisher.publish(message)

    def _publish_goal(self):
        target = self.commanded_goal
        if target is None:
            if self.home_nav is None or self.home_raw is None:
                return
            target = self._nav_target_from_raw(
                (self.home_raw[0], self.home_raw[1], self.goal_altitude)
            )
        if len(target) != 3:
            return
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.position.z = target[2]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _publish_mode(self, mode):
        self._publish_text(self.mode_pub, mode)

    def _command_goal(self, position):
        self.commanded_goal = tuple(position)
        self._publish_goal()

    def _periodic_cargo_setup(self, now):
        if now - self.last_periodic_command < 0.5:
            return
        self.last_periodic_command = now
        if not self.side_door_closed:
            self._publish_text(self.cargo_pub, "left_close")
        elif not self.bottom_door_closed:
            self._publish_text(self.cargo_pub, "bottom_close")
        else:
            self._publish_text(self.cargo_pub, "status")

    def _armed(self):
        return bool(
            self.vehicle_status
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def _offboard(self):
        return bool(
            self.vehicle_status
            and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _auto_land(self):
        return bool(
            self.vehicle_status
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        )

    def _landed(self):
        return bool(self.land_status and self.land_status.landed)

    def _writer_state(self):
        state = {}
        for topic in PX4_INPUT_TOPICS:
            infos = self.get_publishers_info_by_topic(topic)
            state[topic] = normalized_node_names(infos)
        return state

    def _preflight_gates(self, now):
        rates = {name: tracker.rate(now) for name, tracker in self.trackers.items()}
        ages = {name: tracker.age(now) for name, tracker in self.trackers.items()}
        writers = self._writer_state()
        status = self.vehicle_status
        stability = self.stability.report()
        prearm_sample = None
        calibrated_spawn_pose = False
        spawn_position_error = float("inf")
        if self.raw_position is not None:
            prearm_sample = PrearmPoseSample(
                position=tuple(self.raw_position),
                speed_mps=self.raw_speed,
                roll_deg=self.raw_roll,
                pitch_deg=self.raw_pitch,
            )
            calibrated_spawn_pose = prearm_pose_allowed(
                prearm_sample, self.prearm_limits
            )
            spawn_position_error = math.dist(
                prearm_sample.position, self.prearm_limits.expected_position
            )
        # VehicleLandDetected is a discrete transition topic. A late-joining
        # probe may not receive the already-landed sample, so the measured
        # supported-airframe stability is an equivalent pre-arm observation.
        grounded = self._landed() or stability.passes()
        gates = {
            "clock_monotonic": (
                self.clock_monotonic
                and rates["clock"] >= 2.0
                and ages["clock"] <= 0.5
            ),
            "raw_pose": rates["raw_pose"] >= 5.0 and ages["raw_pose"] <= 0.4,
            "raw_twist": rates["raw_twist"] >= 5.0 and ages["raw_twist"] <= 0.4,
            "pointcloud": rates["pointcloud"] >= 2.0 and ages["pointcloud"] <= 0.45,
            "px4_sensor": rates["px4_sensor"] >= 4.0 and ages["px4_sensor"] <= 0.4,
            "px4_odometry": rates["px4_odometry"] >= 4.0 and ages["px4_odometry"] <= 0.4,
            # These PX4 topics are event-driven discrete state, not heartbeats.
            # The sensor and odometry gates above prove that transport is live.
            "px4_status": status is not None,
            "land_status_or_supported": ground_observation_usable(
                self.land_status is not None, stability.passes()
            ),
            "navigation_odometry": self.nav_position is not None and ages["nav_odometry"] <= 0.45,
            "planner_map_ready": planner_map_ready(
                self.planner_state,
                ages["planner_state"],
                self.planner_map_timeout,
            ),
            "fixed_diagnostic_enabled": (
                not self.fixed_setpoint_diagnostic
                or parse_bool_token(
                    self.executor_state, "fixed_setpoint_enabled"
                ) is True
            ),
            "landing_region_verified": verified_landing_region_available(
                self.landing_region, self.landing_region_verified
            ),
            "actuator_feedback_ready": (
                not self.require_live_actuator_feedback
                or (
                    self.have_actuator_motors
                    and rates["actuator_motors"] >= 4.0
                    and ages["actuator_motors"] <= 0.4
                )
            ),
            "px4_ready": bool(status and status.pre_flight_checks_pass and not status.failsafe),
            "disarmed": not self._armed(),
            "landed_or_supported": grounded,
            "side_door_closed": self.side_door_closed,
            "bottom_door_closed": self.bottom_door_closed,
            "payload_locked": self.payload_locked is True,
            "prearm_support": self.prearm_support is True,
            "calibrated_spawn_pose": calibrated_spawn_pose,
            "prearm_attitude_agreement": (
                self.nav_roll is not None
                and self.nav_pitch is not None
                and prearm_attitude_agreement_allowed(
                    self.raw_roll,
                    self.raw_pitch,
                    self.nav_roll,
                    self.nav_pitch,
                    self.prearm_attitude_tolerance_deg,
                )
            ),
            "stable_airframe": stability.passes(),
            "sole_px4_writer": all(
                sole_writer_is(
                    [info for info in self.get_publishers_info_by_topic(topic)],
                    "/trajectory_executor",
                )
                for topic in PX4_INPUT_TOPICS
            ),
        }
        details = {
            "rates_hz": {key: round(value, 3) for key, value in rates.items()},
            "ages_s": {key: round(value, 3) for key, value in ages.items()},
            "stability": {
                "duration_s": round(stability.duration, 3),
                "max_drift_m": round(stability.max_drift_m, 6),
                "max_speed_mps": round(stability.max_speed_mps, 6),
                "max_tilt_deg": round(stability.max_tilt_deg, 4),
            },
            "prearm_pose": {
                "expected_position": self.prearm_limits.expected_position,
                "position_error_m": round(spawn_position_error, 6),
                "position_tolerance_m": self.prearm_limits.position_tolerance_m,
                "speed_mps": round(self.raw_speed, 6),
                "max_speed_mps": self.prearm_limits.max_speed_mps,
                "roll_deg": round(self.raw_roll, 4),
                "pitch_deg": round(self.raw_pitch, 4),
                "max_tilt_deg": self.prearm_limits.max_tilt_deg,
                "px4_roll_deg": (
                    None if self.nav_roll is None else round(self.nav_roll, 4)
                ),
                "px4_pitch_deg": (
                    None if self.nav_pitch is None else round(self.nav_pitch, 4)
                ),
                "attitude_agreement_tolerance_deg": (
                    self.prearm_attitude_tolerance_deg
                ),
            },
            "writers": writers,
            "executor_state": self.executor_state,
            "planner_state": self.planner_state,
            "landing_region": {
                "center_xy": self.landing_region.center_xy,
                "half_extents_xy": self.landing_region.half_extents_xy,
                "verified": self.landing_region_verified,
            },
        }
        return gates, details

    def _set_phase(self, phase):
        self.phase = phase
        self.phase_started = self._now()
        self.phase_started_sim_ns = self.clock_last_ns
        if phase in {"FIXED_HOLD", "HOLD"}:
            self.hold_max_actuator_output = 0.0
            self.hold_saturation_seen = False
        self._event("phase", phase=phase, round=self.current_round)

    def _phase_elapsed(self, now, use_sim_time=False):
        if use_sim_time:
            return phase_elapsed_seconds(
                self.phase_started,
                now,
                self.phase_started_sim_ns,
                self.clock_last_ns,
            )
        return now - self.phase_started

    def _fixed_target_errors(self, target):
        if (
            self.raw_position is None
            or self.home_raw is None
            or self.home_nav is None
        ):
            return float("inf"), float("inf")
        raw_target = self._raw_target_from_nav(target)
        return (
            math.hypot(
                self.raw_position[0] - raw_target[0],
                self.raw_position[1] - raw_target[1],
            ),
            abs(self.raw_position[2] - raw_target[2]),
        )

    def _nav_target_from_raw(self, target):
        return translate_target_between_origins(
            tuple(target), self.home_raw, self.home_nav
        )

    def _raw_target_from_nav(self, target):
        return translate_target_between_origins(
            tuple(target), self.home_nav, self.home_raw
        )

    def _fixed_target_settled(self, target, now):
        horizontal_error, altitude_error = self._fixed_target_errors(target)
        reached = fixed_step_reached(
            horizontal_error,
            altitude_error,
            self.raw_speed,
            horizontal_tolerance_m=self.fixed_target_horizontal_tolerance,
            altitude_tolerance_m=self.fixed_target_altitude_tolerance,
        )
        if reached:
            if self.fixed_settle_since is None:
                self.fixed_settle_since = now
                self.fixed_settle_since_sim_ns = self.clock_last_ns
        else:
            self.fixed_settle_since = None
            self.fixed_settle_since_sim_ns = None
        elapsed = phase_elapsed_seconds(
            self.fixed_settle_since or now,
            now,
            self.fixed_settle_since_sim_ns,
            self.clock_last_ns,
        )
        return reached and elapsed >= self.fixed_step_settle_seconds

    def _stale_flight_topics(self, now):
        ages = {
            name: self.trackers[name].age(now)
            for name in CONTINUOUS_FLIGHT_TOPICS
        }
        return stale_flight_topics(
            ages,
            self.flight_data_timeout,
            self.flight_clock_timeout,
        )

    def _begin_round(self):
        self.current_round += 1
        self.home_nav = tuple(self.nav_position)
        self.home_raw = tuple(self.raw_position)
        self.round_ack_start = len(self.command_acks)
        self.commanded_goal = tuple(self.home_nav)
        self.fixed_step_index = 0
        self.fixed_settle_since = None
        self.fixed_settle_since_sim_ns = None
        self.abort_return_altitude = None
        self.goal_stable_since = None
        self.goal_stable_since_sim_ns = None
        self.hold_max_horizontal_error = 0.0
        self.hold_max_altitude_error = 0.0
        self.hold_max_speed = 0.0
        self.max_yaw_rate_seen = 0.0
        self.hold_max_actuator_output = 0.0
        self.hold_saturation_seen = False
        self._set_phase("ARM")

    def _abort(self, reason, emergency=False):
        if self.phase in {"ABORT", "DONE"}:
            return
        if self.phase == "ABORT_RETURN":
            if emergency:
                self.failure_reason = reason
                self._event(
                    "abort_return_emergency",
                    reason=reason,
                    round=self.current_round,
                )
                self._set_phase("ABORT")
            return
        self.failure_reason = reason
        self._event(
            "abort",
            reason=reason,
            round=self.current_round,
            emergency=emergency,
        )
        recoverable = abort_return_allowed(
            self.fixed_setpoint_diagnostic,
            emergency,
            self._armed(),
            self._offboard(),
            self.home_nav is not None,
            self.raw_position is not None,
            (
                self.raw_position[2]
                if self.raw_position is not None
                else float("nan")
            ),
            self.home_raw[2] if self.home_raw is not None else float("nan"),
            full_horizontal_control=full_horizontal_control_available(
                self.executor_state
            ),
        )
        if recoverable:
            self.abort_return_altitude = max(
                self.home_raw[2] + self.fixed_step_clearances[-1],
                self.raw_position[2],
            )
            self.fixed_settle_since = None
            self.fixed_settle_since_sim_ns = None
            self._set_phase("ABORT_RETURN")
        else:
            self._set_phase("ABORT")

    def _finish(self, success):
        if self.finished:
            return
        if (
            success
            and not self.preflight_only
            and not self.require_live_actuator_feedback
        ):
            success = False
            self.failure_reason = (
                "flight completed; PX4 ULog actuator saturation analysis required"
            )
        self.success = success
        self.finished = True
        self.phase = "DONE"
        self._publish_mode("CLEAR")
        result = {
            "success": success,
            "failure_reason": self.failure_reason,
            "preflight_only": self.preflight_only,
            "requested_rounds": self.rounds,
            "completed_rounds": len(self.round_results),
            "hover_seconds": self.hover_seconds,
            "goal_altitude": self.goal_altitude,
            "fixed_setpoint_diagnostic": self.fixed_setpoint_diagnostic,
            "fixed_step_clearances": self.fixed_step_clearances,
            "fixed_step_timeout": self.fixed_step_timeout,
            "fixed_target_horizontal_tolerance": (
                self.fixed_target_horizontal_tolerance
            ),
            "fixed_target_altitude_tolerance": self.fixed_target_altitude_tolerance,
            "fixed_step_max_speed_mps": self.fixed_step_max_speed_mps,
            "fixed_step_max_tilt_deg": self.fixed_step_max_tilt_deg,
            "fixed_guided_horizontal_limit_m": (
                self.fixed_guided_horizontal_limit_m
            ),
            "fixed_full_xy_horizontal_limit_m": (
                self.fixed_full_xy_horizontal_limit_m
            ),
            "fixed_hold_altitude": self.fixed_hold_altitude,
            "max_yaw_rate_rad_s": self.max_yaw_rate_rad_s,
            "max_yaw_rate_seen_rad_s": round(self.max_yaw_rate_seen, 4),
            "force_disarm_diagnostic_enabled": (
                self.enable_force_disarm_diagnostic
            ),
            "cradle_touchdown": self.cradle_touchdown,
            "landing_region": {
                "center_xy": self.landing_region.center_xy,
                "half_extents_xy": self.landing_region.half_extents_xy,
                "verified": self.landing_region_verified,
            },
            "touchdown_position": self.touchdown_position,
            "command_acks": self.command_acks,
            "live_actuator_feedback_required": self.require_live_actuator_feedback,
            "postflight_ulog_actuator_analysis_required": (
                not self.require_live_actuator_feedback
            ),
            "preflight_gates": self.last_gates,
            "rounds": self.round_results,
            "events": self.events,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(f"evidence={self.output_path} success={success}")

    def _tick(self):
        now = self._now()
        if self.finished:
            return
        if self.phase == "RESET_START":
            self._publish_mode("RESET")
            auto_loiter = bool(
                self.vehicle_status
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            )
            grounded = self._landed() or self.stability.report().passes()
            reset_ready = startup_reset_ready(
                self.executor_lifecycle,
                auto_loiter,
                self._armed(),
                grounded,
            )
            if now - self.last_reset_log >= 2.0:
                self.last_reset_log = now
                self._event(
                    "startup_reset",
                    executor_lifecycle=self.executor_lifecycle,
                    auto_loiter=auto_loiter,
                    armed=self._armed(),
                    landed=self._landed(),
                    grounded=grounded,
                )
            if reset_ready:
                self._publish_mode("CLEAR")
                self._set_phase("WAIT_PREFLIGHT")
            elif now - self.phase_started > 15.0:
                self._abort("startup reset timeout")
            return
        if self.phase == "WAIT_PREFLIGHT":
            self._periodic_cargo_setup(now)
            gates, details = self._preflight_gates(now)
            self.last_gates = {"gates": gates, "details": details}
            if now - self.last_gate_log >= 2.0:
                self.last_gate_log = now
                failed = sorted(name for name, passed in gates.items() if not passed)
                self._event("preflight", failed=failed, details=details)
            if all(gates.values()):
                self._event("preflight_passed", details=details)
                if self.preflight_only:
                    self._finish(True)
                else:
                    self._begin_round()
            return

        if self.vehicle_status and self.vehicle_status.failsafe:
            self._abort("PX4 failsafe active", emergency=True)
        stale_topics = self._stale_flight_topics(now)
        if self.phase not in {"LAND", "RESET", "ABORT"} and stale_topics:
            self._abort(
                "flight telemetry stale for more than "
                f"its configured timeout: {','.join(stale_topics)}",
                emergency=True,
            )
        spin_sensitive_phases = {
            "FIXED_STEP",
            "FIXED_HOLD_SETTLE",
            "FIXED_HOLD",
            "RETURN_HOME",
            "ASCEND",
            "HOLD",
            "ABORT_RETURN",
        }
        if (
            self.phase in spin_sensitive_phases
            and not yaw_rate_envelope_safe(
                self.raw_yaw_rate, self.max_yaw_rate_rad_s
            )
        ):
            self._abort(
                "yaw-rate safety envelope violated: "
                f"rate={self.raw_yaw_rate:.3f}rad/s "
                f"limit={self.max_yaw_rate_rad_s:.3f}rad/s",
                emergency=True,
            )
        if (
            self.phase not in {"LAND", "RESET", "ABORT"}
            and self.executor_lifecycle == "LAND_LATCHED"
        ):
            self._abort("executor entered terminal landing latch", emergency=True)

        if self.phase == "ARM":
            self._publish_goal()
            self._publish_mode(
                "ARM_FIXED" if self.fixed_setpoint_diagnostic else "ARM_OFFBOARD"
            )
            if self._armed() and self._offboard() and self.executor_lifecycle == "ACTIVE":
                self._set_phase(
                    "FIXED_STEP" if self.fixed_setpoint_diagnostic else "ASCEND"
                )
            elif now - self.phase_started > 25.0:
                self._abort("arming/offboard timeout")
        elif self.phase == "FIXED_STEP":
            clearance = self.fixed_step_clearances[self.fixed_step_index]
            target = (
                self.home_nav[0],
                self.home_nav[1],
                self.home_nav[2] + clearance,
            )
            self._command_goal(target)
            self._publish_mode("FIXED")
            horizontal_from_home = math.hypot(
                self.raw_position[0] - self.home_raw[0],
                self.raw_position[1] - self.home_raw[1],
            )
            drop_below_home = max(0.0, self.home_raw[2] - self.raw_position[2])
            max_tilt = max(abs(self.raw_roll), abs(self.raw_pitch))
            full_horizontal_control = full_horizontal_control_available(
                self.executor_state
            )
            max_horizontal_displacement = fixed_step_horizontal_limit(
                full_horizontal_control,
                self.fixed_guided_horizontal_limit_m,
                self.fixed_full_xy_horizontal_limit_m,
            )
            if not fixed_step_envelope_safe(
                horizontal_from_home,
                drop_below_home,
                self.raw_speed,
                max_tilt,
                max_horizontal_displacement_m=max_horizontal_displacement,
                max_speed_mps=self.fixed_step_max_speed_mps,
                max_tilt_deg=self.fixed_step_max_tilt_deg,
            ):
                self._abort(
                    "fixed step safety envelope violated: "
                    f"horizontal={horizontal_from_home:.3f}m "
                    f"horizontal_limit={max_horizontal_displacement:.3f}m "
                    f"full_xy={full_horizontal_control} "
                    f"drop={drop_below_home:.3f}m "
                    f"speed={self.raw_speed:.3f}m/s "
                    f"speed_limit={self.fixed_step_max_speed_mps:.3f}m/s "
                    f"tilt={max_tilt:.1f}deg "
                    f"tilt_limit={self.fixed_step_max_tilt_deg:.1f}deg"
                )
            elif self._fixed_target_settled(target, now):
                self._event(
                    "fixed_step_passed",
                    index=self.fixed_step_index,
                    clearance_m=clearance,
                    target=target,
                )
                (
                    self.fixed_step_index,
                    sequence_complete,
                    next_step_started,
                    next_step_started_sim_ns,
                ) = advance_fixed_step(
                    self.fixed_step_index,
                    len(self.fixed_step_clearances),
                    now,
                    self.clock_last_ns,
                )
                self.fixed_settle_since = None
                self.fixed_settle_since_sim_ns = None
                if sequence_complete:
                    self._set_phase("FIXED_HOLD_SETTLE")
                else:
                    self.phase_started = next_step_started
                    self.phase_started_sim_ns = next_step_started_sim_ns
            elif self._phase_elapsed(now, use_sim_time=True) > self.fixed_step_timeout:
                self._abort(f"fixed step {clearance:.2f} m timeout")
        elif self.phase == "FIXED_HOLD_SETTLE":
            target = self._nav_target_from_raw(
                (
                    self.home_raw[0],
                    self.home_raw[1],
                    self.fixed_hold_altitude,
                )
            )
            self._command_goal(target)
            self._publish_mode("FIXED")
            if self._fixed_target_settled(target, now):
                self._set_phase("FIXED_HOLD")
            elif self._phase_elapsed(now, use_sim_time=True) > 15.0:
                self._abort("fixed hold target timeout")
        elif self.phase == "FIXED_HOLD":
            target = self._nav_target_from_raw(
                (
                    self.home_raw[0],
                    self.home_raw[1],
                    self.fixed_hold_altitude,
                )
            )
            self._command_goal(target)
            self._publish_mode("FIXED")
            horizontal, altitude_error = self._fixed_target_errors(target)
            self.hold_max_horizontal_error = max(
                self.hold_max_horizontal_error, horizontal
            )
            self.hold_max_altitude_error = max(
                self.hold_max_altitude_error, altitude_error
            )
            self.hold_max_speed = max(self.hold_max_speed, self.raw_speed)
            if not hold_acceptance_passes(
                horizontal,
                altitude_error,
                self.raw_speed,
                self.hold_saturation_seen,
                not self.require_live_actuator_feedback
                or self.trackers["actuator_motors"].age(now)
                <= self.actuator_acceptance_timeout,
            ):
                self._abort(
                    "fixed hold acceptance violated: "
                    f"horizontal={horizontal:.3f}m "
                    f"altitude={altitude_error:.3f}m "
                    f"speed={self.raw_speed:.3f}m/s "
                    f"actuator_max={self.hold_max_actuator_output:.3f}"
                )
            elif self._phase_elapsed(now, use_sim_time=True) >= self.hover_seconds:
                self.fixed_settle_since = None
                self.fixed_settle_since_sim_ns = None
                self._set_phase("RETURN_HOME")
        elif self.phase == "RETURN_HOME":
            if self.cradle_touchdown:
                raw_target = cradle_touchdown_target(
                    self.landing_region,
                    self.home_raw[2],
                )
            else:
                raw_target = landing_approach_target(
                    self.landing_region,
                    self.home_raw[2],
                    self.fixed_step_clearances[-1],
                )
            target = self._nav_target_from_raw(raw_target)
            self._command_goal(target)
            self._publish_mode("FIXED")
            inside_region = self.landing_region.contains(tuple(self.raw_position[:2]))
            if inside_region and self._fixed_target_settled(target, now):
                self._set_phase("LAND")
            elif self._phase_elapsed(now, use_sim_time=True) > self.landing_return_timeout:
                self._abort("return to landing region timeout")
        elif self.phase == "ASCEND":
            self._publish_goal()
            self._publish_mode("TRAJECTORY")
            if self.raw_position is None:
                return
            horizontal = math.hypot(
                self.raw_position[0] - self.home_raw[0],
                self.raw_position[1] - self.home_raw[1],
            )
            near_goal = takeoff_goal_reached(
                horizontal,
                self.raw_position[2],
                self.home_raw[2],
                self.goal_altitude,
            )
            if near_goal:
                if self.goal_stable_since is None:
                    self.goal_stable_since = now
                    self.goal_stable_since_sim_ns = self.clock_last_ns
            else:
                self.goal_stable_since = None
                self.goal_stable_since_sim_ns = None
            stable_elapsed = phase_elapsed_seconds(
                self.goal_stable_since or now,
                now,
                self.goal_stable_since_sim_ns,
                self.clock_last_ns,
            )
            if self.goal_stable_since and stable_elapsed >= 2.0:
                self._set_phase("HOLD")
            elif self._phase_elapsed(now, use_sim_time=True) > 15.0:
                self._abort("takeoff goal timeout")
        elif self.phase == "HOLD":
            self._publish_mode("HOLD")
            horizontal = math.hypot(
                self.raw_position[0] - self.home_raw[0],
                self.raw_position[1] - self.home_raw[1],
            )
            altitude_error = abs(self.raw_position[2] - self.goal_altitude)
            self.hold_max_horizontal_error = max(self.hold_max_horizontal_error, horizontal)
            self.hold_max_altitude_error = max(self.hold_max_altitude_error, altitude_error)
            self.hold_max_speed = max(self.hold_max_speed, self.raw_speed)
            if not hold_acceptance_passes(
                horizontal,
                altitude_error,
                self.raw_speed,
                self.hold_saturation_seen,
                not self.require_live_actuator_feedback
                or self.trackers["actuator_motors"].age(now)
                <= self.actuator_acceptance_timeout,
            ):
                self._abort("hover acceptance violated")
            elif self._phase_elapsed(now, use_sim_time=True) >= self.hover_seconds:
                self._set_phase("LAND")
        elif self.phase == "LAND":
            if (
                self.fixed_setpoint_diagnostic
                and self._armed()
                and self.raw_position is not None
                and not self.landing_region.contains(tuple(self.raw_position[:2]))
            ):
                self._abort("left landing region before NAV_LAND")
                return
            self._publish_mode("LAND")
            if self._landed() and not self._armed() and self.executor_lifecycle == "COMPLETE":
                self.touchdown_position = tuple(self.raw_position) if self.raw_position else None
                if (
                    self.touchdown_position is None
                    or not self.landing_region.contains(
                        tuple(self.touchdown_position[:2])
                    )
                ):
                    self._abort("touchdown outside verified landing region")
                    return
                self.round_results.append(
                    {
                        "round": self.current_round,
                        "max_horizontal_error_m": round(self.hold_max_horizontal_error, 4),
                        "max_altitude_error_m": round(self.hold_max_altitude_error, 4),
                        "max_speed_mps": round(self.hold_max_speed, 4),
                        "max_actuator_output": round(
                            self.hold_max_actuator_output, 4
                        ),
                        "actuator_saturation_seen": self.hold_saturation_seen,
                        "touchdown_position": self.touchdown_position,
                        "landed": True,
                        "disarmed": True,
                    }
                )
                self._set_phase("RESET")
            elif (
                self._phase_elapsed(now, use_sim_time=True) > 30.0
                or self._phase_elapsed(now) > self.land_wall_timeout
            ):
                self._abort("landing/disarm timeout")
        elif self.phase == "RESET":
            self._publish_mode("RESET")
            reset_mode_ready = bool(
                self.vehicle_status
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            )
            if self.executor_lifecycle == "DISABLED" and reset_mode_ready:
                round_acks = self.command_acks[self.round_ack_start:]
                successful_acks = [
                    ack for ack in round_acks if successful_command_ack(ack)
                ]
                if self.round_results:
                    self.round_results[-1]["command_acks"] = round_acks
                    self.round_results[-1]["successful_command_ack_count"] = len(
                        successful_acks
                    )
                if len(successful_acks) < 5:
                    self._abort(
                        "fewer than five successful PX4 command ACKs: "
                        f"{len(successful_acks)}"
                    )
                    return
                self._publish_mode("CLEAR")
                if self.current_round >= self.rounds:
                    self._finish(True)
                else:
                    self.stability.clear()
                    self._set_phase("WAIT_PREFLIGHT")
            elif now - self.phase_started > 8.0:
                self._abort("executor reset timeout")
        elif self.phase == "ABORT":
            if self._landed() and not self._armed():
                if self.abort_reset_since is None:
                    self.abort_reset_since = now
                    self._event("abort_reset", round=self.current_round)
                self._publish_mode("RESET")
                if now - self.abort_reset_since >= 1.0:
                    self._finish(False)
            elif (
                now - self.phase_started > self.land_wall_timeout
                and not self.abort_landing_timeout_reported
            ):
                self.abort_landing_timeout_reported = True
                self.failure_reason += "; emergency landing confirmation delayed"
                self._event("abort_landing_delayed", round=self.current_round)
            else:
                crash_candidate = crashed_airframe_force_disarm_ready(
                    self.enable_force_disarm_diagnostic,
                    self._armed(),
                    self._auto_land(),
                    self.executor_lifecycle,
                    self.raw_position[2] if self.raw_position is not None else float("inf"),
                    self.prearm_limits.expected_position[2],
                    self.raw_speed,
                    self.raw_roll,
                    self.raw_pitch,
                    0.0,
                    minimum_stationary_seconds=0.0,
                )
                if crash_candidate:
                    if self.force_disarm_crash_since is None:
                        self.force_disarm_crash_since = now
                    ready = crashed_airframe_force_disarm_ready(
                        self.enable_force_disarm_diagnostic,
                        self._armed(),
                        self._auto_land(),
                        self.executor_lifecycle,
                        self.raw_position[2],
                        self.prearm_limits.expected_position[2],
                        self.raw_speed,
                        self.raw_roll,
                        self.raw_pitch,
                        now - self.force_disarm_crash_since,
                        minimum_stationary_seconds=self.force_disarm_crash_delay,
                    )
                    if ready:
                        if not self.force_disarm_request_logged:
                            self.force_disarm_request_logged = True
                            self._event(
                                "force_disarm_requested", round=self.current_round
                            )
                        self._publish_mode("FORCE_DISARM")
                    else:
                        self._publish_mode("LAND")
                else:
                    self.force_disarm_crash_since = None
                    self.force_disarm_request_logged = False
                    self._publish_mode("LAND")
        elif self.phase == "ABORT_RETURN":
            raw_target = (
                self.landing_region.center_xy[0],
                self.landing_region.center_xy[1],
                self.home_raw[2]
                if self.cradle_touchdown
                else self.abort_return_altitude,
            )
            target = self._nav_target_from_raw(raw_target)
            self._command_goal(target)
            self._publish_mode("FIXED")
            inside_region = self.landing_region.contains(tuple(self.raw_position[:2]))
            if inside_region and self._fixed_target_settled(target, now):
                self._event("abort_return_ready", target=target)
                self._set_phase("ABORT")
            elif self._phase_elapsed(now, use_sim_time=True) > self.landing_return_timeout:
                self._event("abort_return_timeout", target=target)
                self._set_phase("ABORT")


def main(args=None):
    # Keep the ROS context alive on Ctrl-C long enough to publish and latch LAND.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = HoverProbe()
    interrupted = False
    try:
        while rclpy.ok() and not node.finished:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except KeyboardInterrupt:
                if interrupted:
                    node._abort("repeated operator interrupt", emergency=True)
                else:
                    node._abort("operator interrupt")
                    interrupted = True
                # Keep spinning until landed/disarmed. With a low simulator
                # real-time factor, a fixed wall-time grace period can expire
                # while the aircraft is still armed and would orphan ACTIVE.
                continue
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
