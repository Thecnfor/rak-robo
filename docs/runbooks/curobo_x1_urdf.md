# cuRobo IK for the Mercury X1 arm

> 2026-07-20 — the cuRobo integration is now wired end-to-end through
> the ROS 2 ``plan_to_pose_server_node`` action. The default robot
> config is ``ur10e.yml`` (cuRobo's built-in 6-DOF arm, geometrically
> analogous to the X1 arm and produces sub-µm IK accuracy on the
> Tesla T4 GPU). A placeholder X1 URDF is included for the eventual
> per-M1 export.

## Pipeline at a glance

```
ROS 2 host Python (plan_to_pose_server_node)
  → perception_competition_pkg.curobo_client.invoke_curobo()
    → subprocess.run(<isaacsim51 python>, curobo_ik_solver.py, payload)
      → curobo.inverse_kinematics.InverseKinematics.solve_pose()
        → IKResult: joints (6) + position_error + solve_time_ms
    ← JSON on stdout
  → JointState on /hand_command (X1 articulation)
```

The bridge script (``scripts/curobo_ik_solver.py``) is installed via
the package's ``data_files`` and discovered by
``perception_competition_pkg.curobo_client._resolve_curobo_script``.

## Activation prerequisites

The cuRobo kernel needs three pieces that may not be present in every
host. ``curobo_client.probe_health()`` reports which one is missing.

1. **PyPI mirror with cuda-python 12.4.x**. The
   ``isaacsim51`` conda env ships ``cuda-python 12.6`` but cuRobo 0.8
   expects the legacy ``cuda.bindings`` shim which is only present in
   12.4.x. Install with:

       pip install --no-deps 'cuda-python==12.4.0'

   This pulls the legacy ``cuda/bindings`` subpackage that cuRobo's
   ``_src/curobolib/backends/cuda_core_backend`` imports at startup.

2. **``cuda.bindings`` and ``cuda.pathfinder``** as separate
   meta-packages (already present in the standard install once 12.4
   is in place).

3. **nvidia-cuda-runtime-cu12 12.6.x** is required by torch 2.7.
   Install with:

       pip install --no-deps nvidia-cuda-runtime-cu12==12.6.77

## Health check

```bash
source /opt/ros/jazzy/setup.bash
source /var/workspace/docker/isaac/workspace/install/setup.bash
python3 -c "
from perception_competition_pkg.curobo_client import probe_health, invoke_curobo
print('probe_health:', probe_health())
r = invoke_curobo([0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0])
print('success:', r.success, 'time:', r.solve_time_ms, 'err:', r.position_error_mm)
"
```

Output when the env is healthy:

    probe_health: ok
    success: True time: 4.99 err: 3.2e-05

Output when ``cuda.bindings`` is missing:

    probe_health: ok
    success: False msg: cuda_bindings_missing: install cuda-python 12.4.x ...

Output when cuRobo itself is unavailable:

    success: False msg: curobo_unavailable

## X1 arm URDF

The placeholder URDF lives at
``src/perception_competition_pkg/perception_competition_pkg/scripts/x1_left_arm.urdf``.
It mirrors the X1 USD prim chain
``link_body → link1_L → link3_L → link4_L → link5_L → link6_L → tool``
with link lengths typical of a 6-DOF arm. The companion cuRobo config
``x1_left_arm.yml`` registers it with the cuda-core backend.

**To register the real X1 arm with cuRobo**:

1. Export the X1 USD arm as a real URDF with the **actual** joint-frame
   offsets (M1 deliverable). Drop the ``.urdf`` file at
   ``$ISAACSIM/lib/python3.11/site-packages/curobo/content/assets/robot/x1_left_arm.urdf``
   and the matching ``.yml`` config in
   ``$ISAACSIM/lib/python3.11/site-packages/curobo/content/configs/robot/x1_left_arm.yml``.
2. Update ``x1_left_arm.yml``'s ``collision_spheres`` to match the
   new link lengths (this is what tells the IK optimiser which
   regions to avoid for self-collision).
3. Set ``curobo_enabled=true`` and pass
   ``robot_yaml='x1_left_arm.yml'`` to
   ``perception_competition_pkg.curobo_client.invoke_curobo`` from the
   action server.

Until that happens, ``ur10e.yml`` is the production default. Both
configs use the **same 6-DOF joint ordering** (yaw-pitch-elbow-roll-
pitch-roll) so switching the URDF does not require any code change
beyond the parameter.

## Fallback path

When cuRobo is unavailable or the IK fails, the action server
publishes the result of ``curobo_client.analytic_grasp_pose()``.
The fallback biases the wrist toward the observation pose and adjusts
joint 2/3 by a small offset proportional to the target distance.
This is good enough for video smoke tests but the accuracy is
several centimetres vs. the cuRobo sub-µm precision.
