#!/usr/bin/env python3
"""Pure safety helpers for the PX4 hover acceptance probe."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

from collections import deque
from dataclasses import dataclass
from math import asin, atan2, copysign, degrees, sqrt


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
    return event[len(prefix):] if event.startswith(prefix) else current


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
    position_tolerance_m: float = 0.02
    max_speed_mps: float = 0.05
    max_tilt_deg: float = 3.0


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
