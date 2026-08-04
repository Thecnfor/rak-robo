"""Audit recorded stage-2 flight tickets and the ten-run acceptance gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


REQUIRED_PHASES = (
    "IDLE",
    "PREFLIGHT",
    "ARMING",
    "TAKEOFF",
    "EGO_TRANSIT",
    "TARGET_SEARCH",
    "VISUAL_ALIGN",
    "DROP_HOLD",
    "RETURN",
    "LAND",
    "COMPLETE",
)


def _phase_edges(messages: Iterable[str]) -> list[str]:
    edges: list[str] = []
    for message in messages:
        phase = str(message).split(maxsplit=1)[0].strip().upper()
        if phase and (not edges or phase != edges[-1]):
            edges.append(phase)
    nominal: list[str] = []
    for phase in edges:
        if phase == "HOLD":
            continue
        if not nominal or phase != nominal[-1]:
            nominal.append(phase)
    return nominal


def evaluate_run(
    *,
    phase_messages: Iterable[str],
    px4_status_messages: Iterable[str],
    landed_values: Sequence[bool],
    final_payload_xy: tuple[float, float] | None,
    target_xy: tuple[float, float],
    maximum_truth_speed_mps: float | None,
    payload_stationary: bool = True,
) -> dict:
    """Evaluate one physical flight ticket without weakening the 90% drop rule."""
    phases = _phase_edges(phase_messages)
    phase_sequence_ok = phases == list(REQUIRED_PHASES)
    drop_attempted = "DROP_HOLD" in phases
    failsafe_seen = any(
        "failsafe=true" in str(message).lower()
        for message in px4_status_messages
    )
    final_landed = bool(landed_values and landed_values[-1])
    payload_valid = drop_attempted and payload_stationary and final_payload_xy is not None and all(
        math.isfinite(value) for value in (*final_payload_xy, *target_xy)
    )
    drop_error = (
        math.hypot(
            final_payload_xy[0] - target_xy[0],
            final_payload_xy[1] - target_xy[1],
        )
        if payload_valid and final_payload_xy is not None
        else None
    )
    failed_checks = []
    if not phase_sequence_ok:
        failed_checks.append("phase_sequence")
    if failsafe_seen:
        failed_checks.append("px4_failsafe")
    if not final_landed:
        failed_checks.append("final_landed")
    if not payload_valid:
        failed_checks.append("payload_evidence")
    return {
        "passed": not failed_checks,
        "failed_checks": failed_checks,
        "phase_sequence": phases,
        "phase_sequence_ok": phase_sequence_ok,
        "px4_failsafe_seen": failsafe_seen,
        "final_landed": final_landed,
        "drop_attempted": drop_attempted,
        "payload_stationary": payload_stationary,
        "drop_error_m": drop_error,
        "drop_within_0_2_m": drop_error is not None and drop_error <= 0.2,
        "maximum_truth_speed_mps": maximum_truth_speed_mps,
    }


def aggregate_acceptance(runs: Sequence[dict]) -> dict:
    successful_runs = sum(bool(run.get("passed")) for run in runs)
    measured_drops = sum(
        bool(run.get("drop_attempted")) and run.get("drop_error_m") is not None
        for run in runs
    )
    accurate_drops = sum(
        bool(run.get("drop_attempted")) and bool(run.get("drop_within_0_2_m"))
        for run in runs
    )
    drop_rate = accurate_drops / measured_drops if measured_drops else 0.0
    consecutive_successes = 0
    for run in reversed(runs):
        if not run.get("passed"):
            break
        consecutive_successes += 1
    consecutive_flights_ok = consecutive_successes >= 10
    drops_ok = measured_drops >= 10 and drop_rate >= 0.9
    return {
        "passed": consecutive_flights_ok and drops_ok,
        "run_count": len(runs),
        "successful_runs": successful_runs,
        "consecutive_successful_runs": consecutive_successes,
        "ten_consecutive_flights_ok": consecutive_flights_ok,
        "measured_drops": measured_drops,
        "drop_within_0_2_m": accurate_drops,
        "drop_success_rate": drop_rate,
        "drop_acceptance_ok": drops_ok,
    }


def audit_bag(path: Path, target_xy: tuple[float, float]) -> dict:
    """Read one rosbag2 MCAP flight ticket using the installed ROS runtime."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    message_types = {
        name: get_message(type_name) for name, type_name in topic_types.items()
    }
    phases: list[str] = []
    px4_statuses: list[str] = []
    landed: list[bool] = []
    payload_xy = None
    payload_samples = []
    maximum_speed = None
    while reader.has_next():
        topic, serialized, stamp = reader.read_next()
        if topic not in message_types:
            continue
        if topic not in {
            "/drone/navigation/state",
            "/drone/navigation/px4_status",
            "/drone/navigation/landed",
            "/cargo_bay/payload_position",
            "/drone0/state/twist",
        }:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == "/drone/navigation/state":
            phases.append(message.data)
        elif topic == "/drone/navigation/px4_status":
            px4_statuses.append(message.data)
        elif topic == "/drone/navigation/landed":
            landed.append(bool(message.data))
        elif topic == "/cargo_bay/payload_position":
            payload_xy = (float(message.point.x), float(message.point.y))
            payload_samples.append((stamp, *payload_xy))
        elif topic == "/drone0/state/twist":
            linear = message.twist.linear
            speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
            maximum_speed = speed if maximum_speed is None else max(maximum_speed, speed)
    payload_stationary = False
    if payload_samples:
        final_stamp, final_x, final_y = payload_samples[-1]
        final_window = [
            (x, y) for stamp, x, y in payload_samples
            if final_stamp - stamp <= 1_000_000_000
        ]
        payload_stationary = len(final_window) >= 3 and all(
            math.hypot(x - final_x, y - final_y) <= 0.005
            for x, y in final_window
        )
    result = evaluate_run(
        phase_messages=phases,
        px4_status_messages=px4_statuses,
        landed_values=landed,
        final_payload_xy=payload_xy,
        target_xy=target_xy,
        maximum_truth_speed_mps=maximum_speed,
        payload_stationary=payload_stationary,
    )
    result["bag"] = str(path)
    return result


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--target-x", type=float, default=5.5)
    parser.add_argument("--target-y", type=float, default=-3.5)
    parser.add_argument("--output", type=Path)
    parsed = parser.parse_args(args)
    ordered_bags = sorted(parsed.bags, key=lambda path: path.name)
    runs = [
        audit_bag(path, (parsed.target_x, parsed.target_y))
        for path in ordered_bags
    ]
    report = {"schema": "robotac_stage2_acceptance/v1", "runs": runs}
    report["summary"] = aggregate_acceptance(runs)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if parsed.output:
        parsed.output.parent.mkdir(parents=True, exist_ok=True)
        parsed.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
