#!/usr/bin/env python3
"""Pure safety helpers for the PX4 hover acceptance probe."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

from collections import deque
from dataclasses import dataclass
from math import asin, atan2, copysign, degrees, exp, isfinite, remainder, sqrt


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

FIXED_DIAGNOSTIC_OPTIONAL_TOPICS = frozenset(
    {"pointcloud", "planner_state"}
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
    ignored_topics=(),
):
    """Return stale streams while allowing slow simulation clock delivery."""
    ignored = frozenset(ignored_topics)
    if not ignored.issubset(FIXED_DIAGNOSTIC_OPTIONAL_TOPICS):
        raise ValueError(
            "only fixed-diagnostic planner streams may be ignored"
        )
    stale = []
    for name in CONTINUOUS_FLIGHT_TOPICS:
        if name in ignored:
            continue
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


def update_full_horizontal_control_latch(
    latched: bool,
    executor_state: str,
    reset: bool = False,
) -> bool:
    """Remember an explicit XY handoff despite later stale state delivery.

    The executor emits both transition events and periodic lifecycle reports.
    Under heavy simulator load DDS can deliver an older vertical-only report
    after the handoff report.  A flight round therefore treats the first
    explicit ``fixed_vertical_active=false`` as a one-way transition.  The
    caller must reset the latch before each new diagnostic takeoff.
    """
    if reset:
        return False
    return latched or full_horizontal_control_available(executor_state)


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


def live_raw_error_corrected_nav_target(
    raw_target: tuple,
    raw_position: tuple,
    nav_position: tuple,
) -> tuple:
    """Express a raw-truth return error as a live PX4 navigation target.

    MAVLink GPS latitude/longitude quantization is coarser than the 8 mm
    diagnostic launch-guide radius.  A one-time origin translation therefore
    cannot guarantee a safe return to that narrow guide after a long flight.
    This helper is intentionally limited to the acceptance probe's return
    phase: it closes the final approach on simulator truth while the sole PX4
    writer remains ``trajectory_executor``.
    """
    if not all(
        len(value) == 3
        for value in (raw_target, raw_position, nav_position)
    ):
        raise ValueError("live return correction requires three 3D vectors")
    values = (*raw_target, *raw_position, *nav_position)
    if not all(isfinite(value) for value in values):
        raise ValueError("live return correction requires finite vectors")
    return tuple(
        nav_position[index] + raw_target[index] - raw_position[index]
        for index in range(3)
    )


def landing_nav_target(
    raw_target: tuple,
    raw_position: tuple,
    nav_position: tuple,
    frozen_offset: tuple | None,
    horizontal_gain: float = 1.0,
) -> tuple:
    """Use truth error during approach and an equivalent frozen final target.

    Before capture, adding the simulator-truth position error to the current
    PX4 estimate removes estimator-origin drift from the outer position loop.
    At capture, ``nav_position - raw_position`` produces exactly the same
    target, so switching to a stationary setpoint has no discontinuity.
    """
    if (
        not isfinite(horizontal_gain)
        or horizontal_gain <= 0.0
        or horizontal_gain > 3.0
    ):
        raise ValueError("landing truth horizontal gain must be in (0, 3]")
    if frozen_offset is None:
        base_target = live_raw_error_corrected_nav_target(
            raw_target,
            raw_position,
            nav_position,
        )
        return (
            nav_position[0]
            + horizontal_gain * (raw_target[0] - raw_position[0]),
            nav_position[1]
            + horizontal_gain * (raw_target[1] - raw_position[1]),
            base_target[2],
        )
    if len(raw_target) != 3 or len(frozen_offset) != 3:
        raise ValueError("landing target and frozen offset must be 3D vectors")
    values = (*raw_target, *frozen_offset)
    if not all(isfinite(value) for value in values):
        raise ValueError("landing target and frozen offset must be finite")
    return tuple(
        raw_target[index] + frozen_offset[index] for index in range(3)
    )


def low_pass_frame_offset(
    previous_offset: tuple,
    observed_offset: tuple,
    elapsed_seconds: float,
    time_constant_seconds: float = 2.0,
    maximum_rate_mps: float = 0.02,
) -> tuple:
    """Track slow estimator-origin drift without commanding sample-time jitter.

    Isaac truth and PX4 odometry are delivered at different rates and their
    message stamps use different clock domains.  Directly subtracting the
    latest two samples can therefore create centimetre-scale target jumps.
    This first-order filter follows the slowly varying frame offset in
    simulation time, then limits the 3D correction rate as a second guard.
    """
    if len(previous_offset) != 3 or len(observed_offset) != 3:
        raise ValueError("frame offsets must be three-dimensional")
    values = (
        *previous_offset,
        *observed_offset,
        elapsed_seconds,
        time_constant_seconds,
        maximum_rate_mps,
    )
    if (
        not all(isfinite(value) for value in values)
        or elapsed_seconds < 0.0
        or time_constant_seconds <= 0.0
        or maximum_rate_mps <= 0.0
    ):
        raise ValueError("frame-offset filter requires finite positive limits")
    if elapsed_seconds == 0.0:
        return tuple(previous_offset)

    alpha = 1.0 - exp(-elapsed_seconds / time_constant_seconds)
    update = tuple(
        alpha * (observed_offset[index] - previous_offset[index])
        for index in range(3)
    )
    update_norm = sqrt(sum(value * value for value in update))
    maximum_step = maximum_rate_mps * elapsed_seconds
    if update_norm > maximum_step:
        scale = maximum_step / update_norm
        update = tuple(value * scale for value in update)
    return tuple(
        previous_offset[index] + update[index] for index in range(3)
    )


def freeze_landing_frame_offset(
    frozen_offset: tuple | None,
    live_offset: tuple | None,
    raw_home: tuple,
    nav_home: tuple,
) -> tuple:
    """Take one estimator-frame snapshot and keep the landing goal stationary.

    The live offset is useful while climbing because it tracks slow PX4
    estimator-origin drift. Continuing to apply it during final approach moves
    the setpoint by centimetres and prevents an 8 mm guide from seeing a
    settled target. The first return sample is therefore frozen for both normal
    and abort-return paths. A measured home-frame translation is the fallback
    when no filtered sample is available.
    """
    candidate = frozen_offset
    if candidate is None:
        candidate = live_offset
    if candidate is None:
        if len(raw_home) != 3 or len(nav_home) != 3:
            raise ValueError("landing frame origins must be three-dimensional")
        candidate = tuple(
            nav_home[index] - raw_home[index] for index in range(3)
        )
    if len(candidate) != 3 or not all(isfinite(value) for value in candidate):
        raise ValueError("landing frame offset must be a finite 3D vector")
    return tuple(candidate)


def landing_frame_capture_action(
    frozen: bool,
    horizontal_error_m: float,
    speed_mps: float,
    capture_radius_m: float = 0.008,
    release_radius_m: float = 0.012,
    maximum_capture_speed_mps: float = 0.05,
) -> str:
    """Select live correction or a stationary final-approach frame.

    The PX4-to-Isaac frame offset may drift by more than the launch guide
    clearance during a long return. Follow the filtered live offset until
    simulator truth is already inside the touchdown gate, then freeze it long
    enough to prove a settled static setpoint. Hysteresis returns to live
    correction if the aircraft moves materially outside the gate.
    """
    values = (
        horizontal_error_m,
        speed_mps,
        capture_radius_m,
        release_radius_m,
        maximum_capture_speed_mps,
    )
    if (
        not all(isfinite(value) for value in values)
        or horizontal_error_m < 0.0
        or speed_mps < 0.0
        or capture_radius_m <= 0.0
        or release_radius_m <= capture_radius_m
        or maximum_capture_speed_mps <= 0.0
    ):
        raise ValueError("landing-frame capture requires finite hysteresis limits")
    if frozen:
        return "release" if horizontal_error_m > release_radius_m else "hold"
    if (
        horizontal_error_m <= capture_radius_m
        and speed_mps <= maximum_capture_speed_mps
    ):
        return "capture"
    return "live"


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


def cradle_approach_target(
    region: LandingRegion,
    home_z: float,
    clearance_m: float,
) -> tuple:
    """Return a centered pose above the guide for a PX4 LAND handoff."""
    if not region.valid() or not isfinite(home_z) or clearance_m <= 0.0:
        raise ValueError("cradle approach requires valid region and clearance")
    return (
        region.center_xy[0],
        region.center_xy[1],
        home_z + clearance_m,
    )


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


def fixed_clearance_sequence_valid(
    clearances_m,
    hold_altitude_m: float,
    home_altitude_m: float,
    authorized_max_clearance_m: float = 0.30,
    hard_max_clearance_m: float = 0.70,
) -> bool:
    """Validate an explicitly authorized, monotonically increasing climb ladder."""
    values = tuple(float(value) for value in clearances_m)
    if (
        not values
        or not isfinite(hold_altitude_m)
        or not isfinite(home_altitude_m)
        or not isfinite(authorized_max_clearance_m)
        or not isfinite(hard_max_clearance_m)
        or authorized_max_clearance_m <= 0.0
        or hard_max_clearance_m <= 0.0
        or authorized_max_clearance_m > hard_max_clearance_m
    ):
        return False
    hold_clearance = hold_altitude_m - home_altitude_m
    return (
        0.0 < hold_clearance <= authorized_max_clearance_m
        and all(
            isfinite(clearance)
            and 0.0 < clearance <= authorized_max_clearance_m
            for clearance in values
        )
        and all(later > earlier for earlier, later in zip(values, values[1:]))
    )


def guided_touchdown_reached(
    horizontal_error_m: float,
    altitude_error_m: float,
    speed_mps: float,
    guide_radius_m: float = 0.008,
    altitude_tolerance_m: float = 0.05,
    max_speed_mps: float = 0.05,
) -> bool:
    """Require 2 mm clearance inside the current 10 mm launch guide."""
    if not 0.0 < guide_radius_m <= 0.008:
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
