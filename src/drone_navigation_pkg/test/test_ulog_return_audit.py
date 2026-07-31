#!/usr/bin/env python3
"""Regression tests for the PX4 static-return ULog audit."""

# Copyright 2026 Competition Team
# SPDX-License-Identifier: GPL-3.0-only

import importlib.util
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "ulog_return_audit.py"
)
SPEC = importlib.util.spec_from_file_location("ulog_return_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class UlogReturnAuditTest(unittest.TestCase):
    def test_continuous_settle_window_passes(self):
        result = AUDIT.summarize_return_trace(
            times_s=[0.0, 0.5, 1.0, 1.5, 2.0],
            positions_xy=[
                (0.007, 0.0),
                (0.006, 0.0),
                (0.005, 0.0),
                (0.004, 0.0),
                (0.003, 0.0),
            ],
            velocities_xy=[(0.01, 0.0)] * 5,
            target_xy=(0.0, 0.0),
            required_settle_s=1.5,
        )
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["longest_settle_s"], 2.0)

    def test_low_frequency_orbit_reproduces_failed_return(self):
        result = AUDIT.summarize_return_trace(
            times_s=[0.0, 0.5, 1.0, 1.5, 2.0, 2.5],
            positions_xy=[
                (0.006, 0.0),
                (0.007, 0.0),
                (0.012, 0.0),
                (0.020, 0.0),
                (0.007, 0.0),
                (0.006, 0.0),
            ],
            velocities_xy=[(0.01, 0.0)] * 6,
            target_xy=(0.0, 0.0),
            required_settle_s=1.5,
        )
        self.assertFalse(result["passed"])
        self.assertAlmostEqual(result["longest_settle_s"], 0.5)
        self.assertGreater(result["maximum_horizontal_error_m"], 0.008)

    def test_non_monotonic_timestamps_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            AUDIT.summarize_return_trace(
                times_s=[0.0, 0.0],
                positions_xy=[(0.0, 0.0)] * 2,
                velocities_xy=[(0.0, 0.0)] * 2,
                target_xy=(0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main()
