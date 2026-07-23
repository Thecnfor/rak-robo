#!/usr/bin/env python3
"""Audit PX4 actual and requested attitude envelopes from a persisted ULog."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import argparse
import json
import math
from pathlib import Path


def quaternion_roll_pitch_degrees(quaternion):
    """Return roll and pitch for a PX4 [w, x, y, z] quaternion."""
    if len(quaternion) != 4:
        raise ValueError("attitude quaternion must have four elements")
    values = tuple(float(value) for value in quaternion)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("attitude quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("attitude quaternion must have non-zero norm")
    w, x, y, z = (value / norm for value in values)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.atan2(sin_roll, cos_roll)), math.degrees(
        math.asin(sin_pitch)
    )


def summarize_attitude_trace(
    actual_quaternions,
    setpoint_quaternions,
    maximum_tilt_deg=15.0,
):
    """Summarize the exact low-altitude tilt gate used by the flight probe."""
    if not math.isfinite(maximum_tilt_deg) or maximum_tilt_deg <= 0.0:
        raise ValueError("maximum tilt must be positive")
    if not actual_quaternions or not setpoint_quaternions:
        raise ValueError("actual and setpoint attitude traces are required")
    actual = [
        quaternion_roll_pitch_degrees(quaternion)
        for quaternion in actual_quaternions
    ]
    setpoints = [
        quaternion_roll_pitch_degrees(quaternion)
        for quaternion in setpoint_quaternions
    ]
    actual_roll = max(abs(sample[0]) for sample in actual)
    actual_pitch = max(abs(sample[1]) for sample in actual)
    setpoint_roll = max(abs(sample[0]) for sample in setpoints)
    setpoint_pitch = max(abs(sample[1]) for sample in setpoints)
    return {
        "passed": max(actual_roll, actual_pitch) <= maximum_tilt_deg,
        "maximum_tilt_deg": maximum_tilt_deg,
        "actual_max_abs_roll_deg": actual_roll,
        "actual_max_abs_pitch_deg": actual_pitch,
        "setpoint_max_abs_roll_deg": setpoint_roll,
        "setpoint_max_abs_pitch_deg": setpoint_pitch,
        "actual_samples": len(actual),
        "setpoint_samples": len(setpoints),
    }


def _quaternions(dataset, field):
    return list(
        zip(
            dataset.data[f"{field}[0]"],
            dataset.data[f"{field}[1]"],
            dataset.data[f"{field}[2]"],
            dataset.data[f"{field}[3]"],
        )
    )


def audit_ulog(path, maximum_tilt_deg=15.0):
    """Load the two PX4 attitude streams and apply the deterministic gate."""
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
            "vehicle_attitude",
            "vehicle_attitude_setpoint",
        ],
    )
    datasets = {dataset.name: dataset for dataset in ulog.data_list}
    missing = sorted(
        name
        for name in ("vehicle_attitude", "vehicle_attitude_setpoint")
        if name not in datasets
    )
    if missing:
        raise ValueError("ULog is missing attitude streams: " + ", ".join(missing))
    result = summarize_attitude_trace(
        _quaternions(datasets["vehicle_attitude"], "q"),
        _quaternions(datasets["vehicle_attitude_setpoint"], "q_d"),
        maximum_tilt_deg,
    )
    result["ulog"] = str(Path(path).resolve())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--maximum-tilt-deg", type=float, default=15.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit_ulog(args.ulog, args.maximum_tilt_deg)
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
