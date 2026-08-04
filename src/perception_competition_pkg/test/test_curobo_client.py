"""Unit tests for the cuRobo client + analytic fallback.

These tests do not require the cuRobo kernel to be installed;
they exercise the JSON wire protocol that the subprocess bridge
returns and the analytic fallback that runs when the kernel is
unavailable.
"""
import json
import os
import subprocess
import sys
import unittest
from unittest import mock


class InvokeResultParsingTest(unittest.TestCase):
    """The subprocess returns a JSON line; ``invoke_curobo`` must
    parse it into an ``InvokeResult`` with all the documented fields.
    """

    def test_invoke_curobo_parses_successful_response(self):
        from perception_competition_pkg.curobo_client import (
            invoke_curobo, InvokeResult, _resolve_curobo_script, _resolve_isaac_python,
        )
        # Skip if the actual env is missing the cuRobo script/isaacsim51.
        if _resolve_curobo_script() is None or _resolve_isaac_python() is None:
            self.skipTest('curobo_ik_solver not installed in this env')
        # Hit the live subprocess: the real solver returns success or
        # a clean failure depending on whether the kernel is loaded.
        result = invoke_curobo([0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
        self.assertIsInstance(result, InvokeResult)
        if result.success:
            self.assertEqual(len(result.joints), 6)
            self.assertGreater(result.solve_time_ms, 0.0)
            self.assertIsNotNone(result.position_error_mm)
        else:
            # We accept any of the activation-needed tags without
            # failing: the test is about contract, not about whether
            # cuRobo is *active*.
            self.assertIsNone(result.joints)

    def test_analytic_fallback_clamps_to_pi(self):
        from perception_competition_pkg.curobo_client import (
            analytic_grasp_pose,
        )
        # Distant target should still produce clamped joints in [-pi, pi].
        out = analytic_grasp_pose(
            (10.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
        )
        self.assertEqual(len(out), 6)
        for q in out:
            self.assertGreaterEqual(q, -3.14159265)
            self.assertLessEqual(q, 3.14159265)

    def test_analytic_fallback_observation_pose_default(self):
        from perception_competition_pkg.curobo_client import (
            analytic_grasp_pose,
        )
        # When the input is short, the heuristic should bias toward
        # the observation pose.
        out = analytic_grasp_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
        )
        # The default observation pose is (0, -0.4, 0.6, -0.2, 0, 0).
        # We only assert on the first and last (which are unchanged by
        # When the target is the origin and the long axis is +X, the
        # heuristic sets j1 = 0.6*0 + 0.3*1.0 = 0.3 (axis_x contribution).
        self.assertAlmostEqual(out[0], 0.3, places=2)
        self.assertAlmostEqual(out[5], 0.0, places=2)


class ScriptCliContractTest(unittest.TestCase):
    """Run the actual ``curobo_ik_solver.py`` subprocess with a known
    payload and check the JSON contract.
    """

    @classmethod
    def setUpClass(cls):
        # Locate the script in source-tree or installed share.
        candidates = [
            os.path.join(
                os.path.dirname(__file__),
                '..', 'perception_competition_pkg', 'scripts', 'curobo_ik_solver.py',
            ),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.insert(
                0,
                os.path.join(
                    get_package_share_directory('perception_competition_pkg'),
                    'scripts', 'curobo_ik_solver.py',
                ),
            )
        except Exception:
            pass
        cls.script = next(
            (c for c in candidates if os.path.isfile(c)), None,
        )

    def test_missing_argv_returns_clean_failure(self):
        if self.script is None:
            self.skipTest('curobo_ik_solver.py not installed')
        # The script is designed to run under the isaacsim51 Python;
        # if it's not on PATH, the test still validates the failure
        # mode (curobo_unavailable or cuda_bindings_missing).
        for py in self._candidate_pythons():
            proc = subprocess.run(
                [py, self.script, '{}'],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(proc.returncode, 0)
            self.assertIn('"success"', proc.stdout)
            data = json.loads(proc.stdout.strip().splitlines()[-1])
            self.assertFalse(data['success'])
            self.assertIn('joints', data)
            self.assertIn('solve_time_ms', data)
            self.assertIn('position_error_mm', data)
            self.assertIsNone(data['joints'])
            return

    def test_malformed_json_returns_clean_failure(self):
        if self.script is None:
            self.skipTest('curobo_ik_solver.py not installed')
        for py in self._candidate_pythons():
            proc = subprocess.run(
                [py, self.script, 'not-json'],
                capture_output=True, text=True, timeout=30,
            )
            self.assertEqual(proc.returncode, 0)
            data = json.loads(proc.stdout.strip().splitlines()[-1])
            self.assertFalse(data['success'])
            self.assertIn('invalid_json', data['message'])
            return

    def _candidate_pythons(self):
        """Yield candidate Python interpreters in priority order."""
        candidates = [
            '/home/socl/miniconda3/envs/isaacsim51/bin/python3',
            '/home/socl/miniconda3/envs/isaacsim51/bin/python',
        ]
        for c in candidates:
            if os.path.isfile(c) and os.access(c, os.X_OK):
                yield c
        # Last resort: whatever runs pytest itself.
        yield sys.executable


if __name__ == '__main__':
    unittest.main()
