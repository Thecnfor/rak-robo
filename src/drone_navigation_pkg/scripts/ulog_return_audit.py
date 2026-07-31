#!/usr/bin/env python3
"""Audit PX4 tracking during the final static return before NAV_LAND."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import argparse
import json
import math
from pathlib import Path


def _percentile(values, percentile):
    ordered = sorted(values)
    if not ordered:
        raise ValueError("at least one sample is required")
    index = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def summarize_return_trace(
    times_s,
    positions_xy,
    velocities_xy,
    target_xy,
    maximum_horizontal_error_m=0.008,
    maximum_horizontal_speed_mps=0.05,
    required_settle_s=1.5,
):
    """Measure the longest continuously settled interval at a static target.

    The flight probe remains the source of truth because its acceptance uses
    Isaac ground truth. This ULog check is a deterministic PX4-side proxy that
    catches the same low-frequency orbit between powered trials.
    """
    times = [float(value) for value in times_s]
    positions = [tuple(float(value) for value in sample) for sample in positions_xy]
    velocities = [
        tuple(float(value) for value in sample) for sample in velocities_xy
    ]
    target = tuple(float(value) for value in target_xy)
    if not times or len(times) != len(positions) or len(times) != len(velocities):
        raise ValueError("return traces must be non-empty and have equal lengths")
    if len(target) != 2 or any(len(sample) != 2 for sample in positions + velocities):
        raise ValueError("return position, velocity and target samples must be 2D")
    flattened = times + list(target)
    flattened.extend(value for sample in positions + velocities for value in sample)
    if not all(math.isfinite(value) for value in flattened):
        raise ValueError("return trace values must be finite")
    if any(later <= earlier for earlier, later in zip(times, times[1:])):
        raise ValueError("return timestamps must be strictly increasing")
    if (
        maximum_horizontal_error_m <= 0.0
        or maximum_horizontal_speed_mps <= 0.0
        or required_settle_s <= 0.0
    ):
        raise ValueError("return acceptance thresholds must be positive")

    errors = [
        math.hypot(position[0] - target[0], position[1] - target[1])
        for position in positions
    ]
    speeds = [math.hypot(*velocity) for velocity in velocities]
    longest_settle = 0.0
    settle_start = None
    for timestamp, error, speed in zip(times, errors, speeds):
        within_gate = (
            error <= maximum_horizontal_error_m
            and speed <= maximum_horizontal_speed_mps
        )
        if within_gate:
            if settle_start is None:
                settle_start = timestamp
            longest_settle = max(longest_settle, timestamp - settle_start)
        else:
            settle_start = None

    return {
        "passed": longest_settle >= required_settle_s,
        "maximum_horizontal_error_m": max(errors),
        "p95_horizontal_error_m": _percentile(errors, 95.0),
        "maximum_horizontal_speed_mps": max(speeds),
        "p95_horizontal_speed_mps": _percentile(speeds, 95.0),
        "longest_settle_s": longest_settle,
        "required_settle_s": required_settle_s,
        "horizontal_error_limit_m": maximum_horizontal_error_m,
        "horizontal_speed_limit_mps": maximum_horizontal_speed_mps,
        "samples": len(times),
    }


def _relative_times(dataset, start_timestamp):
    return [
        (float(timestamp) - float(start_timestamp)) / 1_000_000.0
        for timestamp in dataset.data["timestamp"]
    ]


def _static_return_window(ulog, trajectory, commands):
    trajectory_times = _relative_times(trajectory, ulog.start_timestamp)
    command_times = _relative_times(commands, ulog.start_timestamp)
    land_times = [
        timestamp
        for timestamp, command in zip(command_times, commands.data["command"])
        if timestamp >= 0.0 and int(command) == 21
    ]
    if not land_times:
        raise ValueError("ULog has no external NAV_LAND command")
    end_time = min(land_times)
    candidates = [
        index
        for index, timestamp in enumerate(trajectory_times)
        if 0.0 <= timestamp < end_time
    ]
    if not candidates:
        raise ValueError("ULog has no trajectory setpoint before NAV_LAND")
    last = candidates[-1]
    target = tuple(
        float(trajectory.data[f"position[{axis}]"][last]) for axis in range(3)
    )
    if not all(math.isfinite(value) for value in target):
        raise ValueError("final pre-land trajectory target is not finite")

    first = last
    while first > 0:
        index = first - 1
        position = tuple(
            float(trajectory.data[f"position[{axis}]"][index])
            for axis in range(3)
        )
        velocity = tuple(
            float(trajectory.data[f"velocity[{axis}]"][index])
            for axis in range(3)
        )
        acceleration = tuple(
            float(trajectory.data[f"acceleration[{axis}]"][index])
            for axis in range(3)
        )
        if (
            not all(math.isfinite(value) for value in position + velocity + acceleration)
            or math.dist(position, target) > 1e-6
            or math.sqrt(sum(value * value for value in velocity)) > 1e-6
            or math.sqrt(sum(value * value for value in acceleration)) > 1e-6
        ):
            break
        first = index
    return trajectory_times[first], end_time, target


def audit_ulog(
    path,
    maximum_horizontal_error_m=0.008,
    maximum_horizontal_speed_mps=0.05,
    required_settle_s=1.5,
):
    """Load a ULog and audit its final static-return tracking interval."""
    try:
        from pyulog import ULog
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "pyulog is required to audit PX4 ULogs; install it in the "
            "active ROS environment"
        ) from error

    ulog = ULog(
        str(path),
        message_name_filter_list=[
            "trajectory_setpoint",
            "vehicle_local_position",
            "vehicle_command",
        ],
    )
    datasets = {dataset.name: dataset for dataset in ulog.data_list}
    required = (
        "trajectory_setpoint",
        "vehicle_local_position",
        "vehicle_command",
    )
    missing = [name for name in required if name not in datasets]
    if missing:
        raise ValueError("ULog is missing return streams: " + ", ".join(missing))
    start_time, end_time, target = _static_return_window(
        ulog,
        datasets["trajectory_setpoint"],
        datasets["vehicle_command"],
    )
    local_position = datasets["vehicle_local_position"]
    local_times = _relative_times(local_position, ulog.start_timestamp)
    indices = [
        index
        for index, timestamp in enumerate(local_times)
        if start_time <= timestamp < end_time
    ]
    if len(indices) < 2:
        raise ValueError("static return window has fewer than two position samples")
    result = summarize_return_trace(
        [local_times[index] for index in indices],
        [
            (
                local_position.data["x"][index],
                local_position.data["y"][index],
            )
            for index in indices
        ],
        [
            (
                local_position.data["vx"][index],
                local_position.data["vy"][index],
            )
            for index in indices
        ],
        target[:2],
        maximum_horizontal_error_m,
        maximum_horizontal_speed_mps,
        required_settle_s,
    )
    result.update(
        {
            "ulog": str(Path(path).resolve()),
            "static_return_start_s": start_time,
            "static_return_end_s": end_time,
            "static_return_duration_s": end_time - start_time,
            "target_ned_m": target,
        }
    )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--maximum-horizontal-error-m", type=float, default=0.008)
    parser.add_argument("--maximum-horizontal-speed-mps", type=float, default=0.05)
    parser.add_argument("--required-settle-s", type=float, default=1.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit_ulog(
        args.ulog,
        args.maximum_horizontal_error_m,
        args.maximum_horizontal_speed_mps,
        args.required_settle_s,
    )
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
