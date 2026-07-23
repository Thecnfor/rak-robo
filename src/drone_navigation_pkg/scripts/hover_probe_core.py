#!/usr/bin/env python3
"""Pure safety helpers for the PX4 hover acceptance probe."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

from collections import deque
from dataclasses import dataclass
from math import asin, atan2, copysign, degrees, isfinite, remainder, sqrt


CONTINUOUS_FLIGHT_TOPICS = (
    "clock",
    "raw_pose",
    "raw_twist",
    "pointcloud",
    "px4_sensor",
    "px4_odometry",
    "nav_odometry",
    "planner_state",
)


def successful_command_ack(text: str) -> bool:
    """Accept only an explicitly successful PX4 command acknowledgement."""
    tokens = {}
    for token in text.split():
        if "=" in token:
            key, value = token.split("=", 1)
            tokens[key] = value
    return "command" in tokens and tokens.get("result") == "0"


def stale_flight_topics(
    ages: dict,
    telemetry_timeout: float,
    clock_timeout: float,
):
    """Return stale streams while allowing slow simulation clock delivery."""
    stale = []
    for name in CONTINUOUS_FLIGHT_TOPICS:
        timeout = clock_timeout if name == "clock" else telemetry_timeout
        if ages.get(name, float("inf")) > timeout:
            stale.append(name)
    return stale


def phase_elapsed_seconds(start_wall, now_wall, start_sim_ns, now_sim_ns):
    """Measure a simulated phase in simulation time, with a wall-time fallback."""
    if (
        start_sim_ns is not None
        and now_sim_ns is not None
        and now_sim_ns >= start_sim_ns
    ):
        return (now_sim_ns - start_sim_ns) / 1_000_000_000.0
    return now_wall - start_wall


def update_executor_lifecycle(current: str, event: str) -> str:
    """Latch lifecycle state while preserving it across diagnostic event messages."""
    prefix = "LIFECYCLE "
    if not event.startswith(prefix):
        return current
    fields = event[len(prefix):].split()
    return fields[0] if fields else current


def parse_bool_token(text: str, key: str):
    """Return a key=value boolean, or None when the token is absent/invalid."""
    prefix = f"{key.lower()}="
    for token in text.lower().split():
        if not token.startswith(prefix):
            continue
        value = token[len(prefix):]
        if value in {"true", "1", "yes"}:
            return True
        if value in {"false", "0", "no"}:
            return False
    return None


def full_horizontal_control_available(executor_state: str) -> bool:
    """Trust XY recovery only after the executor explicitly reports handoff."""
    return parse_bool_token(executor_state, "fixed_vertical_active") is False


def planner_map_ready(state: str, state_age_seconds: float, timeout: float) -> bool:
    """Require a fresh planner state that proves a transformed map update."""
    return (
        timeout >= 0.0
        and state_age_seconds >= 0.0
        and state_age_seconds <= timeout
        and parse_bool_token(state, "map_ready") is True
    )


def quaternion_roll_pitch_degrees(x: float, y: float, z: float, w: float):
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = (
        copysign(0.5 * 3.141592653589793, sin_pitch)
        if abs(sin_pitch) >= 1.0
        else asin(sin_pitch)
    )
    return degrees(roll), degrees(pitch)


def crashed_airframe_force_disarm_ready(
    enabled: bool,
    armed: bool,
    auto_land: bool,
    executor_lifecycle: str,
    current_z: float,
    spawn_z: float,
    speed_mps: float,
    roll_deg: float,
    pitch_deg: float,
    stationary_duration_s: float,
    minimum_drop_m: float = 0.25,
    maximum_speed_mps: float = 0.05,
    minimum_tilt_deg: float = 90.0,
    minimum_stationary_seconds: float = 1.0,
) -> bool:
    """Recognize an unrecoverable, stationary crash after AUTO_LAND.

    This is intentionally much stricter than ordinary landed detection.  It is
    only an operator diagnostic fallback for the case where PX4 keeps motors
    armed because a flipped vehicle was not classified as landed.
    """
    values = (
        current_z,
        spawn_z,
        speed_mps,
        roll_deg,
        pitch_deg,
        stationary_duration_s,
    )
    return (
        enabled
        and armed
        and auto_land
        and executor_lifecycle == "LAND_LATCHED"
        and all(isfinite(value) for value in values)
        and minimum_drop_m > 0.0
        and maximum_speed_mps >= 0.0
        and minimum_tilt_deg > 0.0
        and minimum_stationary_seconds >= 0.0
        and current_z <= spawn_z - minimum_drop_m
        and speed_mps <= maximum_speed_mps
        and max(abs(roll_deg), abs(pitch_deg)) >= minimum_tilt_deg
        and stationary_duration_s >= minimum_stationary_seconds
    )


def prearm_attitude_agreement_allowed(
    raw_roll_deg: float,
    raw_pitch_deg: float,
    estimated_roll_deg: float,
    estimated_pitch_deg: float,
    maximum_error_deg: float = 0.5,
) -> bool:
    """Require the PX4 tilt estimate to agree with simulator ground truth."""
    values = (
        raw_roll_deg,
        raw_pitch_deg,
        estimated_roll_deg,
        estimated_pitch_deg,
        maximum_error_deg,
    )
    if not all(isfinite(value) for value in values) or maximum_error_deg < 0.0:
        return False
    roll_error = abs(remainder(estimated_roll_deg - raw_roll_deg, 360.0))
    pitch_error = abs(estimated_pitch_deg - raw_pitch_deg)
    return roll_error <= maximum_error_deg and pitch_error <= maximum_error_deg


def abort_return_allowed(
    fixed_setpoint_diagnostic: bool,
    emergency: bool,
    armed: bool,
    offboard: bool,
    has_home: bool,
    has_raw_position: bool,
    current_z: float,
    home_z: float,
    full_horizontal_control: bool = False,
    minimum_airborne_clearance_m: float = 0.03,
) -> bool:
    """Allow XY return only when airborne with horizontal control available.

    Keeping a climb setpoint active while the aircraft still touches its guide
    lets the vertical controller build thrust after an abort. Likewise, a
    vertical-only setpoint cannot arrest horizontal drift after leaving the
    guide. In either case the probe must request LAND immediately; only an
    airborne aircraft with full XY control may return to the landing region.
    """
    return (
        fixed_setpoint_diagnostic
        and not emergency
        and armed
        and offboard
        and has_home
        and has_raw_position
        and full_horizontal_control
        and isfinite(current_z)
        and isfinite(home_z)
        and isfinite(minimum_airborne_clearance_m)
        and minimum_airborne_clearance_m >= 0.0
        and current_z - home_z >= minimum_airborne_clearance_m
    )


def translate_target_between_origins(
    target: tuple,
    source_origin: tuple,
    destination_origin: tuple,
) -> tuple:
    """Translate one map-aligned target between two measured origins.

    PX4 local position and Isaac ground truth share ENU axes but can have a
    small estimator-origin offset.  Commands belong in the PX4-adapted map;
    acceptance limits belong in Isaac ground truth.  Keeping this translation
    explicit prevents that offset from becoming a false altitude or landing
    error.
    """
    if not all(
        len(value) == 3
        for value in (target, source_origin, destination_origin)
    ):
        raise ValueError("target translation requires three 3D vectors")
    values = (*target, *source_origin, *destination_origin)
    if not all(isfinite(value) for value in values):
        raise ValueError("target translation requires finite vectors")
    return tuple(
        destination_origin[index] + target[index] - source_origin[index]
        for index in range(3)
    )


class RateWindow:
    def __init__(self, horizon_seconds: float = 5.0):
        self._horizon = horizon_seconds
        self._samples = deque()

    def add(self, now_seconds: float):
        self._samples.append(now_seconds)
        self._trim(now_seconds)

    def _trim(self, now_seconds: float):
        while self._samples and now_seconds - self._samples[0] > self._horizon:
            self._samples.popleft()

    def age(self, now_seconds: float):
        return float("inf") if not self._samples else now_seconds - self._samples[-1]

    def rate(self, now_seconds: float):
        self._trim(now_seconds)
        if len(self._samples) < 2:
            return 0.0
        elapsed = self._samples[-1] - self._samples[0]
        return 0.0 if elapsed <= 0.0 else (len(self._samples) - 1) / elapsed


@dataclass(frozen=True)
class StabilityReport:
    duration: float
    max_drift_m: float
    max_speed_mps: float
    max_tilt_deg: float

    def passes(
        self,
        duration_seconds: float = 3.0,
        max_drift_m: float = 0.02,
        max_speed_mps: float = 0.05,
        max_tilt_deg: float = 3.0,
    ) -> bool:
        return (
            self.duration >= duration_seconds
            and self.max_drift_m <= max_drift_m
            and self.max_speed_mps <= max_speed_mps
            and self.max_tilt_deg <= max_tilt_deg
        )


@dataclass(frozen=True)
class PrearmPoseSample:
    position: tuple
    speed_mps: float
    roll_deg: float
    pitch_deg: float


@dataclass(frozen=True)
class PrearmPoseLimits:
    expected_position: tuple
    position_tolerance_m: float = 0.004
    max_speed_mps: float = 0.05
    max_tilt_deg: float = 3.0


@dataclass(frozen=True)
class LandingRegion:
    center_xy: tuple
    half_extents_xy: tuple

    def valid(self) -> bool:
        return (
            len(self.center_xy) == 2
            and len(self.half_extents_xy) == 2
            and all(isfinite(value) for value in self.center_xy)
            and all(
                isfinite(value) and value > 0.0
                for value in self.half_extents_xy
            )
        )

    def contains(self, position_xy: tuple) -> bool:
        if not self.valid() or len(position_xy) != 2:
            return False
        return all(
            abs(position_xy[index] - self.center_xy[index])
            <= self.half_extents_xy[index] + 1e-9
            for index in range(2)
        )


def verified_landing_region_available(region: LandingRegion, verified: bool) -> bool:
    """A numerically valid rectangle is not evidence that its surface is safe."""
    return bool(verified) and region.valid()


def landing_approach_target(
    region: LandingRegion,
    home_z: float,
    clearance_m: float,
) -> tuple:
    """Return a fixed approach point above the verified landing region."""
    if not region.valid() or not isfinite(home_z) or clearance_m <= 0.0:
        raise ValueError("landing approach requires valid region and clearance")
    return (region.center_xy[0], region.center_xy[1], home_z + clearance_m)


def cradle_touchdown_target(
    region: LandingRegion,
    home_z: float,
) -> tuple:
    """Return the support contact pose for a guided vertical touchdown."""
    if not region.valid() or not isfinite(home_z):
        raise ValueError("cradle touchdown requires a valid region and home height")
    return (region.center_xy[0], region.center_xy[1], home_z)


def actuator_outputs_saturated(
    outputs,
    saturation_threshold: float = 0.95,
) -> bool:
    """Treat any finite normalized motor output at the configured limit as saturated."""
    if not 0.0 < saturation_threshold <= 1.0:
        return True
    finite_outputs = [abs(value) for value in outputs if isfinite(value)]
    return not finite_outputs or max(finite_outputs) >= saturation_threshold


def hold_acceptance_passes(
    horizontal_error_m: float,
    altitude_error_m: float,
    speed_mps: float,
    saturation_seen: bool,
    actuator_feedback_fresh: bool = True,
    horizontal_tolerance_m: float = 0.10,
    altitude_tolerance_m: float = 0.05,
    max_speed_mps: float = 0.05,
) -> bool:
    """Require every sample in the timed HOLD window to satisfy the exit gate."""
    return (
        0.0 <= horizontal_error_m <= horizontal_tolerance_m
        and 0.0 <= altitude_error_m <= altitude_tolerance_m
        and 0.0 <= speed_mps <= max_speed_mps
        and not saturation_seen
        and actuator_feedback_fresh
    )


def prearm_pose_allowed(
    sample: PrearmPoseSample,
    limits: PrearmPoseLimits,
) -> bool:
    """Require the disarmed vehicle to occupy its calibrated support envelope."""
    if len(sample.position) != 3 or len(limits.expected_position) != 3:
        return False
    if (
        limits.position_tolerance_m < 0.0
        or limits.max_speed_mps < 0.0
        or limits.max_tilt_deg < 0.0
    ):
        return False
    position_error = sqrt(
        sum(
            (sample.position[index] - limits.expected_position[index]) ** 2
            for index in range(3)
        )
    )
    return (
        position_error <= limits.position_tolerance_m
        and sample.speed_mps <= limits.max_speed_mps
        and abs(sample.roll_deg) <= limits.max_tilt_deg
        and abs(sample.pitch_deg) <= limits.max_tilt_deg
    )


def fixed_step_reached(
    horizontal_error_m: float,
    altitude_error_m: float,
    speed_mps: float,
    horizontal_tolerance_m: float = 0.10,
    altitude_tolerance_m: float = 0.05,
    max_speed_mps: float = 0.05,
) -> bool:
    """Require a fixed target to settle before advancing the step ladder."""
    return (
        0.0 <= horizontal_error_m <= horizontal_tolerance_m
        and 0.0 <= altitude_error_m <= altitude_tolerance_m
        and 0.0 <= speed_mps <= max_speed_mps
    )


def guided_touchdown_reached(
    horizontal_error_m: float,
    altitude_error_m: float,
    speed_mps: float,
    guide_radius_m: float = 0.004,
    altitude_tolerance_m: float = 0.05,
    max_speed_mps: float = 0.05,
) -> bool:
    """Require the body to be centered inside the launch guide before LAND."""
    if not 0.0 < guide_radius_m <= 0.004:
        return False
    return fixed_step_reached(
        horizontal_error_m,
        altitude_error_m,
        speed_mps,
        horizontal_tolerance_m=guide_radius_m,
        altitude_tolerance_m=altitude_tolerance_m,
        max_speed_mps=max_speed_mps,
    )


def advance_fixed_step(
    current_index: int,
    total_steps: int,
    now_wall_seconds: float,
    now_sim_nanoseconds,
):
    """Advance the ladder and give every remaining step a fresh deadline."""
    next_index = current_index + 1
    complete = next_index >= total_steps
    if complete:
        return next_index, True, None, None
    return next_index, False, now_wall_seconds, now_sim_nanoseconds


def fixed_step_envelope_safe(
    horizontal_displacement_m: float,
    drop_below_home_m: float,
    speed_mps: float,
    max_tilt_deg_seen: float,
    max_horizontal_displacement_m: float = 0.20,
    max_drop_below_home_m: float = 0.10,
    max_speed_mps: float = 0.50,
    max_tilt_deg: float = 20.0,
) -> bool:
    """Bound the low-altitude diagnostic before it can leave the table."""
    return (
        0.0 <= horizontal_displacement_m <= max_horizontal_displacement_m
        and 0.0 <= drop_below_home_m <= max_drop_below_home_m
        and 0.0 <= speed_mps <= max_speed_mps
        and 0.0 <= max_tilt_deg_seen <= max_tilt_deg
    )


def fixed_step_horizontal_limit(
    full_horizontal_control: bool,
    guided_limit_m: float = 0.012,
    full_control_limit_m: float = 0.20,
) -> float:
    """Use an inside-guide limit until executor handoff is explicit."""
    if (
        not isfinite(guided_limit_m)
        or not isfinite(full_control_limit_m)
        or guided_limit_m <= 0.0
        or full_control_limit_m <= guided_limit_m
    ):
        raise ValueError("horizontal limits require 0 < guided < full control")
    return full_control_limit_m if full_horizontal_control else guided_limit_m


def yaw_rate_envelope_safe(yaw_rate_rad_s: float, max_yaw_rate_rad_s: float) -> bool:
    """Reject an uncontrolled spin before attempting any lateral recovery."""
    return (
        isfinite(yaw_rate_rad_s)
        and isfinite(max_yaw_rate_rad_s)
        and max_yaw_rate_rad_s > 0.0
        and abs(yaw_rate_rad_s) <= max_yaw_rate_rad_s
    )


class StabilityWindow:
    def __init__(self, horizon_seconds: float = 4.0):
        self._horizon = horizon_seconds
        self._samples = deque()

    def clear(self):
        self._samples.clear()

    def add(self, now_seconds, position, speed_mps, roll_deg, pitch_deg):
        self._samples.append(
            (now_seconds, tuple(position), speed_mps, roll_deg, pitch_deg)
        )
        while self._samples and now_seconds - self._samples[0][0] > self._horizon:
            self._samples.popleft()

    def report(self):
        if not self._samples:
            return StabilityReport(0.0, float("inf"), float("inf"), float("inf"))
        first_time, first_position, _, _, _ = self._samples[0]
        last_time = self._samples[-1][0]
        max_drift = 0.0
        max_speed = 0.0
        max_tilt = 0.0
        for _, position, speed, roll, pitch in self._samples:
            drift = sqrt(sum((position[i] - first_position[i]) ** 2 for i in range(3)))
            max_drift = max(max_drift, drift)
            max_speed = max(max_speed, speed)
            max_tilt = max(max_tilt, abs(roll), abs(pitch))
        return StabilityReport(last_time - first_time, max_drift, max_speed, max_tilt)


def normalized_node_names(endpoint_infos):
    names = set()
    for info in endpoint_infos:
        namespace = (info.node_namespace or "/").rstrip("/")
        namespace = "" if namespace == "/" else namespace
        names.add(f"{namespace}/{info.node_name}")
    return sorted(names)


def sole_writer_is(endpoint_infos, expected_name: str) -> bool:
    return normalized_node_names(endpoint_infos) == [expected_name]


def startup_reset_ready(
    executor_lifecycle: str,
    auto_loiter: bool,
    armed: bool,
    grounded: bool,
) -> bool:
    """Require a fully reset, grounded control chain before a new flight."""
    return (
        executor_lifecycle == "DISABLED"
        and auto_loiter
        and not armed
        and grounded
    )


def takeoff_goal_reached(
    horizontal_error_m: float,
    altitude_m: float,
    home_altitude_m: float,
    goal_altitude_m: float,
    horizontal_tolerance_m: float = 0.15,
    altitude_tolerance_m: float = 0.15,
    minimum_clearance_m: float = 0.20,
) -> bool:
    """Accept a takeoff goal only after physical clearance is established."""
    return (
        horizontal_error_m <= horizontal_tolerance_m
        and abs(altitude_m - goal_altitude_m) <= altitude_tolerance_m
        and altitude_m >= home_altitude_m + minimum_clearance_m
    )


def ground_observation_usable(
    land_status_received: bool,
    support_stable: bool,
) -> bool:
    """Allow a stable static support to replace a missed discrete land sample."""
    return land_status_received or support_stable
