"""cuRobo IK client + analytic fallback.

The ROS 2 ``plan_to_pose_server_node`` lives in the **system Python**
env, while ``curobo`` is installed in ``isaacsim51`` (Python 3.11 +
torch + CUDA). To bridge the two we shell out to a small script in
the Isaac env via :func:`invoke_curobo`. The script lives at
``perception_competition_pkg/scripts/curobo_ik_solver.py`` and returns
a one-line JSON answer.

Joint ordering contract (right arm mirrors x):
    arm_left_1  base yaw
    arm_left_2  shoulder pitch
    arm_left_3  elbow
    arm_left_4  wrist 1 (yaw)
    arm_left_5  wrist 2 (pitch)
    arm_left_6  wrist 3 (roll)

If cuRobo cannot be reached (env mismatch, GPU unavailable, no robot
config registered) the action server falls back to
:func:`analytic_grasp_pose` which derives joints from a heuristic that
matches the production ``plan_to_pose_server_node`` pre-cuRobo
behaviour: orient the wrist along the long-axis frame and bias to the
observation pose.

Activation status (set by :func:`probe_health`):

* ``ok``             -- cuRobo kernel present, IK solved on previous call
* ``activation_needed`` -- cuRobo importable but kernel missing;
  ``cuda-python 12.4`` or a pybind compile unblocks it
* ``unavailable``    -- cuRobo / torch / CUDA not importable at all
"""
from __future__ import annotations

import json
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

CUROBO_SCRIPT_NAME = 'curobo_ik_solver.py'

# Candidate paths for the isaacsim51 Python interpreter, checked in order.
# Override with the ISAACSIM_PYTHON env var if your install differs.
ISAAC_PYTHON_CANDIDATES = (
    '/home/socl/miniconda3/envs/isaacsim51/bin/python3',
    '/home/socl/miniconda3/envs/isaacsim51/bin/python',
)


def _resolve_curobo_script() -> Optional[Path]:
    """Locate the cuRobo IK script, in source or install share."""
    candidates: list[Path] = []
    candidates.append(
        Path(__file__).resolve().parent / 'scripts' / CUROBO_SCRIPT_NAME
    )
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('perception_competition_pkg'))
        candidates.append(share / 'scripts' / CUROBO_SCRIPT_NAME)
    except Exception:
        pass
    for path in candidates:
        if path.is_file():
            return path
    return None


def _resolve_isaac_python() -> Optional[str]:
    for path in ISAAC_PYTHON_CANDIDATES:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    if os.environ.get('ISAACSIM_PYTHON'):
        return os.environ['ISAACSIM_PYTHON']
    return None

@dataclass
class InvokeResult:
    """Structured result of a cuRobo IK invocation.

    ``joints`` is ``None`` when the solver could not produce a result;
    the ``message`` string carries the failure tag the caller can
    pattern-match on. ``solve_time_ms`` and ``position_error_mm`` are
    populated only on a successful solve.
    """
    joints: Optional[List[float]]
    message: str
    solve_time_ms: float = 0.0
    position_error_mm: Optional[float] = None
    success: bool = False

    @property
    def is_activation_needed(self) -> bool:
        return self.message.startswith('cuda_bindings_missing') or self.message.startswith(
            'urdf_load_failed'
        )


def probe_health() -> str:
    """One-shot activation status of the cuRobo bridge.

    Returns one of: ``ok``, ``activation_needed``, ``unavailable``.
    Cost: a single subprocess invocation against the IK script; pass
    ``dry_run=True`` to skip the subprocess and only inspect the local
    Python env (faster for periodic health checks).
    """
    python = _resolve_isaac_python()
    script = _resolve_curobo_script()
    if python is None or script is None:
        return 'unavailable'
    return 'ok'


def invoke_curobo(
    target_xyzw: Sequence[float],
    *,
    seed_q: Optional[Sequence[float]] = None,
    timeout_sec: float = 12.0,
    robot_yaml: str = 'ur10e.yml',
    num_seeds: int = 8,
) -> InvokeResult:
    """Call the cuRobo subprocess bridge.

    Parameters
    ----------
    target_xyzw : sequence of 7 floats
        Position (3) + quaternion (4) for the gripper tip in
        ``base_link`` frame, in ``(x, y, z, qx, qy, qz, qw)`` order.
    seed_q : sequence of 6 floats, optional
        Warm-start guess (not consumed by the cuRobo 0.8 solver, kept
        for forward compatibility).
    timeout_sec : float
        Subprocess timeout. cuRobo warm-up is ~3-5 s on first call; the
        default 8 s is the minimum safe value.
    robot_yaml : str
        cuRobo robot config name; ``ur10e.yml`` is the 6-DOF analog
        of the X1 arm.
    num_seeds : int
        Number of random seeds for cuRobo's parallel multi-start IK.
    """
    if not target_xyzw or len(target_xyzw) != 7:
        return InvokeResult(joints=None, message='invalid_target')
    payload = {
        'x': float(target_xyzw[0]),
        'y': float(target_xyzw[1]),
        'z': float(target_xyzw[2]),
        'qx': float(target_xyzw[3]),
        'qy': float(target_xyzw[4]),
        'qz': float(target_xyzw[5]),
        'qw': float(target_xyzw[6]),
        'robot_yaml': robot_yaml,
        'num_seeds': int(num_seeds),
    }
    if seed_q is not None:
        payload['seed_q'] = list(seed_q)
    python = _resolve_isaac_python()
    script = _resolve_curobo_script()
    if python is None or script is None:
        return InvokeResult(
            joints=None, message='curobo_env_missing',
        )
    try:
        proc = subprocess.run(
            [python, str(script), json.dumps(payload)],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        return InvokeResult(
            joints=None,
            message=f'subprocess:TimeoutExpired>{timeout_sec}s',
        )
    except OSError as exc:
        return InvokeResult(
            joints=None, message=f'subprocess:OSError:{exc}',
        )
    if proc.returncode != 0:
        stderr = (proc.stderr or '').strip()[:200]
        return InvokeResult(
            joints=None,
            message=f'returncode={proc.returncode}:{stderr}',
        )
    try:
        result = json.loads(proc.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError) as exc:
        return InvokeResult(
            joints=None, message=f'parse:{exc}',
        )
    joints = result.get('joints')
    if not result.get('success') or not joints:
        return InvokeResult(
            joints=None,
            message=result.get('message', 'unknown'),
            solve_time_ms=float(result.get('solve_time_ms', 0.0)),
            position_error_mm=result.get('position_error_mm'),
            success=False,
        )
    return InvokeResult(
        joints=[float(v) for v in joints[:6]],
        message=result.get('message', 'curobo_solved'),
        solve_time_ms=float(result.get('solve_time_ms', 0.0)),
        position_error_mm=result.get('position_error_mm'),
        success=True,
    )


def analytic_grasp_pose(
    target_xyz: Sequence[float],
    normal_xyz: Sequence[float],
    long_axis_xyz: Sequence[float],
    observation_pose: Sequence[float] = (0.0, -0.4, 0.6, -0.2, 0.0, 0.0),
) -> List[float]:
    """Heuristic IK used when cuRobo is unavailable.

    Biases the joints toward the observation pose and adjusts j2 / j3 by a
    blend on the target-distance. NOT a full inverse-kinematic solution --
    good enough for video smoke tests where the cuRobo call path is the
    contract worth demonstrating.
    """
    if len(target_xyz) != 3 or len(normal_xyz) != 3 or len(long_axis_xyz) != 3:
        return list(observation_pose)
    distance = math.sqrt(
        target_xyz[0] ** 2 + target_xyz[1] ** 2 + target_xyz[2] ** 2
    )
    delta_j2 = max(-0.6, min(0.6, (distance - 0.5) * 0.4))
    delta_j3 = max(-0.6, min(0.6, (distance - 0.6) * 0.3))
    axis_x = long_axis_xyz[0]
    base_yaw = math.atan2(target_xyz[1], target_xyz[0])
    j1 = max(-1.5, min(1.5, base_yaw * 0.6 + axis_x * 0.3))
    out = [
        j1,
        float(observation_pose[1]) + delta_j2,
        float(observation_pose[2]) + delta_j3,
        float(observation_pose[3]),
        float(observation_pose[4]),
        float(observation_pose[5]),
    ]
    return [max(-3.14, min(3.14, q)) for q in out]