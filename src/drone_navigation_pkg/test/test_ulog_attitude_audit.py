#!/usr/bin/env python3
"""Regression tests for the PX4 ULog attitude-envelope audit."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "ulog_attitude_audit.py"
)
SPEC = importlib.util.spec_from_file_location("ulog_attitude_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def pitch_quaternion(degrees):
    half = math.radians(degrees) * 0.5
    return (math.cos(half), 0.0, math.sin(half), 0.0)


class UlogAttitudeAuditTest(unittest.TestCase):
    def test_stable_trace_passes_tilt_envelope(self):
        result = AUDIT.summarize_attitude_trace(
            [pitch_quaternion(0.0), pitch_quaternion(4.0)],
            [pitch_quaternion(0.0), pitch_quaternion(3.0)],
            maximum_tilt_deg=15.0,
        )
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["actual_max_abs_pitch_deg"], 4.0)
        self.assertAlmostEqual(result["setpoint_max_abs_pitch_deg"], 3.0)

    def test_oscillating_trace_reproduces_failed_flight(self):
        result = AUDIT.summarize_attitude_trace(
            [pitch_quaternion(0.0), pitch_quaternion(21.6)],
            [pitch_quaternion(0.0), pitch_quaternion(5.9)],
            maximum_tilt_deg=15.0,
        )
        self.assertFalse(result["passed"])
        self.assertGreater(result["actual_max_abs_pitch_deg"], 20.0)
        self.assertLess(result["setpoint_max_abs_pitch_deg"], 6.0)


if __name__ == "__main__":
    unittest.main()
