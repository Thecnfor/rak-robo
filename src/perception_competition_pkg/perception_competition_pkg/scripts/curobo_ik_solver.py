#!/usr/bin/env python3
"""cuRobo IK subprocess bridge for the X1 arm.

Run inside the Isaac Sim Python env (``isaacsim51``) where cuRobo
+ torch + CUDA live; the ROS 2 ``plan_to_pose_server_node`` shells out
to this script via ``subprocess.run`` and parses the JSON answer.

The IK target is a 7-tuple ``(x, y, z, qx, qy, qz, qw)`` in the
``base_link_arm`` frame. The script loads a 6-DOF UR10e config as a
6-DOF analog of the X1 arm (the cuRobo-bundled ``ur10e.yml`` matches
the X1 arm joint count and 6-DOF structure). The eventual
**production** path is to register a real X1 URDF; see
``docs/runbooks/curobo_x1_urdf.md``.

Output (one line of JSON on stdout):

    {"joints": [float * 6] | null,
     "success": bool,
     "message": str,
     "solve_time_ms": float,
     "position_error_mm": float | null}

Status messages the ROS 2 caller can pattern-match:

    ok                 -- IK solved
    curobo_unavailable -- no cuRobo module on path
    cuda_bindings_missing -- cuRobo loaded but cuda.bindings not present
    urdf_load_failed   -- could not parse / instantiate the URDF
    no_solution        -- solver ran but no seed converged
    exception:<type>:<message>  -- any other error
"""
from __future__ import annotations

import json
import sys
import time
from typing import Optional


def _emit(payload: dict) -> int:
    sys.stdout.write(json.dumps(payload, ensure_ascii=False))
    sys.stdout.write('\n')
    return 0


_FAILURE = {
    'joints': None,
    'success': False,
    'message': 'curobo_unavailable',
    'solve_time_ms': 0.0,
    'position_error_mm': None,
}


def _import_curobo():
    """Return the cuRobo helpers we need, or ``(None, None, ...)`` on failure."""
    try:
        import torch  # noqa: F401
        from curobo.inverse_kinematics import InverseKinematics  # type: ignore
        from curobo.inverse_kinematics import InverseKinematicsCfg  # type: ignore
        from curobo.types import GoalToolPose, Pose  # type: ignore
        return InverseKinematics, InverseKinematicsCfg, GoalToolPose, Pose
    except Exception as exc:  # pragma: no cover
        return None, None, None, None


def _check_cuda_bindings() -> Optional[str]:
    """Return an error message if cuda.bindings is missing, else None."""
    try:
        import cuda.bindings  # type: ignore  # noqa: F401
        return None
    except Exception:
        return (
            'cuda_bindings_missing: install cuda-python 12.4.x with '
            '`pip install cuda-python==12.4.0 -i https://pypi.org/simple` '
            'or compile cuRobo pybind from source.'
        )


def main(argv: list) -> int:
    started = time.monotonic()
    if len(argv) < 2:
        return _emit({**_FAILURE, 'message': 'missing_argv'})

    try:
        payload = json.loads(argv[1])
    except json.JSONDecodeError as exc:
        return _emit({**_FAILURE, 'message': f'invalid_json:{exc}'})

    InverseKinematics, InverseKinematicsCfg, GoalToolPose, Pose = _import_curobo()
    if InverseKinematics is None:
        return _emit({**_FAILURE, 'message': 'curobo_unavailable'})

    binding_err = _check_cuda_bindings()
    if binding_err is not None:
        return _emit({**_FAILURE, 'message': binding_err})

    x = float(payload.get('x', 0.0))
    y = float(payload.get('y', 0.0))
    z = float(payload.get('z', 0.0))
    qx = float(payload.get('qx', 0.0))
    qy = float(payload.get('qy', 0.0))
    qz = float(payload.get('qz', 0.0))
    qw = float(payload.get('qw', 1.0))
    seed_q = payload.get('seed_q')  # optional 6-vector
    robot_yaml = str(payload.get('robot_yaml', 'ur10e.yml'))
    num_seeds = int(payload.get('num_seeds', 8))
    max_attempts = int(payload.get('max_attempts', 1))
    position_tolerance = float(payload.get('position_tolerance', 0.005))
    orientation_tolerance = float(payload.get('orientation_tolerance', 0.05))

    try:
        import torch  # local import keeps availability check explicit
        config = InverseKinematicsCfg.create(
            robot=robot_yaml,
            num_seeds=num_seeds,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
        ik = InverseKinematics(config)
        target_link = ik.tool_frames[0]
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        goal_pose = Pose(
            position=torch.tensor(
                [[x, y, z]], device=device, dtype=torch.float32,
            ),
            quaternion=torch.tensor(
                [[qw, qx, qy, qz]], device=device, dtype=torch.float32,
            ),
        )
        result = ik.solve_pose(
            GoalToolPose.from_poses({target_link: goal_pose}, num_goalset=1),
        )
        if hasattr(result, 'success') and bool(result.success):
            # Prefer ``result.js_solution`` when the cuRobo build exposes
            # it (it returns a ``JointState`` object whose ``position``
            # tensor is already shaped ``(batch, goalset, n_joints)``);
            # fall back to ``result.solution`` for older 0.7.x builds.
            sol = getattr(result, 'js_solution', None)
            if sol is None or not hasattr(sol, 'position'):
                sol = result.solution
            tensor = getattr(sol, 'position', sol)
            joint_list = tensor.detach().cpu().flatten().tolist()
            joint_list = [float(v) for v in joint_list]
            if len(joint_list) < 6:
                joint_list = joint_list + [0.0] * (6 - len(joint_list))
            joint_list = joint_list[:6]
            position_error_mm = None
            err = getattr(result, 'position_error', None)
            if err is not None:
                try:
                    norm = err.norm() if hasattr(err, 'norm') else err
                    val = norm.item() if hasattr(norm, 'item') else float(norm)
                    position_error_mm = float(val) * 1000.0
                except Exception:
                    position_error_mm = None
            solve_time_ms = float(getattr(result, 'solve_time', 0.0))
            return _emit({
                'joints': joint_list,
                'success': True,
                'message': f'curobo_solved via {robot_yaml}',
                'solve_time_ms': solve_time_ms,
                'position_error_mm': position_error_mm,
            })
        return _emit({
            **_FAILURE,
            'message': 'no_solution',
            'solve_time_ms': float(getattr(result, 'solve_time', 0.0)),
        })
    except Exception as exc:
        return _emit({
            **_FAILURE,
            'message': f'{type(exc).__name__}:{exc}',
            'solve_time_ms': (time.monotonic() - started) * 1000.0,
        })


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
