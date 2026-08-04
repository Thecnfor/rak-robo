# Repository Guidelines

> AI-assistant oriented guide. The hand-curated human onboarding is `CLAUDE.md`
> at the repo root — read it first. `docs/README.md` is the Chinese reading-path
> index for the `docs/` tree. **This file does not duplicate task statuses** —
> those live in `docs/project/tasks_full.md` and `docs/project/submission_checklist.md`.

## Project Overview

ROS 2 Jazzy `colcon` workspace at `/var/workspace/docker/isaac/workspace` for
the 25th ROBOTAC AIROBOTIC pre-selection. The same workspace drives two
pipelines plus Nav2:

- **Manipulation (ground)**: YOLOE → depth pose → TF → arm plan → gripper.
- **Aerial**: local planner → 20 Hz Offboard executor → flight supervisor.
- **Nav2**: lifecycle-managed map/AMCL + planner/controller.

External sims live outside the repo:
- **Isaac Sim 5.1** + **PegasusSimulator** on the host (conda env `isaacsim51`).
- **PX4 v1.16.2 SITL** in Docker at `/var/workspace/docker/isaac/docker/px4/`
  (external-sim mode; Pegasus TCP **4560** → uXRCE-DDS UDP **8888** → host Fast
  DDS domain **45** → `/fmu/*`).

Top-level layout:

- `src/` — 11 ROS 2 packages (colcon source root, no nesting).
- `build/`, `install/`, `log/` — generated, git-ignored.
- `bags/` — ad-hoc rosbag captures, git-ignored.
- `docs/` — setup, runbooks, contracts, project tracking.
- `CLAUDE.md` — engineer onboarding.
- `AGENTS.md` — this file.
- External assets and `/var/workspace/docker/isaac/docker/px4/` live outside the
  repo.

## Architecture & Data Flow

### Package boundaries

| Package | Type | Owns |
|---|---|---|
| `isaac_ros2_control` | ament_python | Host↔Isaac round-trip starters (`cmd_vel_relay`, `tf_echo_bridge`). Observability only — Isaac's OmniGraph drives actuators. |
| `grasp_demo_interfaces` | ament_cmake | Custom actions: `DetectObject`, `PlanToPose`, `GripperCommand`, `DroneFlightCommand`, `CargoDoorCommand`. |
| `grasp_demo_pkg` | ament_python | Teaching grasp pipeline: YOLOE detector, depth pose, TF, plan-to-pose, gripper, pick/place state machine. |
| `nav2_demo_pkg` | ament_cmake | Nav2 launch/config/maps; wraps the installed Nav2 stack. |
| `dual_arm_pkg` (A) | ament_python | Mercury X1 dual-arm + dual-gripper ground state machine. |
| `perception_competition_pkg` (B) | ament_python | Competition vision: YOLOE, depth pose, drone down-look target detection. |
| `bridge_competition_pkg` (C) | ament_python | Host bringup, runtime interface audit, Foxglove, mutually-exclusive direct-rotor smoke test. **Does not plan, does not publish PX4 setpoints.** |
| `drone_navigation_pkg` (D) | ament_cmake (C++17, **GPL-3.0-only**) | PX4 NED/FRD adaptation, isolated voxel/A*/B-spline planning core, 20 Hz Offboard executor (the **only** writer of `/fmu/in/*`), flight safety supervisor. |
| `competition_orchestrator_pkg` (D) | ament_python | Air-ground coordinator, cargo-door actions, drone mission actions. |
| `px4_msgs` | third-party | Vendored PX4 messages. Keep on `release/1.16`. |

Owners: A=dual-arm, B=perception, C=host bringup, D=drone + air-ground.

### Three data flows

**Grasp (teaching pipeline)** — wired by `src/grasp_demo_pkg/launch/perception_pipeline_demo.launch.py`, topics in `src/grasp_demo_pkg/config/demo_params.yaml`:

```
/arm_camera/rgb
  → yoloe_detector_node
  → /demo_grasp/{object_mask, bbox, label, confidence, debug_image}

/arm_camera/depth + camera_info + bbox/mask
  → depth_pose_estimator_node
  → /demo_grasp/{object_point_camera, long_axis_camera}

camera-frame point/axis + TF
  → tf_transform_demo_node
  → /demo_grasp/{object_point_base, normal_base, long_axis_base}

pick_place_state_machine
  → /demo_detect_object action   (DetectObject)
  → /demo_plan_to_pose action    (PlanToPose)
```

YOLOE weights at `src/grasp_demo_pkg/weights/` (git-ignored, deployed separately).

**Nav2** — frame chain `map → odom → base_link`, simulation time, consumes
`/laser_scan_fuse`. Lifecycle-managed map server + AMCL start first; planner,
controller, behavior, BT servers start after a `TimerAction(12s)` delay. Helper
scripts in `src/nav2_demo_pkg/scripts/`.

**Drone chain** — one C++17 library `flight_core` plus four nodes wired in
series by `src/drone_navigation_pkg/launch/navigation.launch.py` and
`src/drone_navigation_pkg/config/navigation.yaml`:

```
/fmu/out/*  (PX4 SITL, BEST_EFFORT QoS)
  → px4_state_adapter   (NED/FRD → ENU/FLU, applies px4_map_origin)
  → /drone/navigation/odometry  + /drone/navigation/px4_status
  → ego_local_planner          (rolling voxel / dynamic A* / uniform B-spline)
  → /drone/navigation/{trajectory, planned_path, planner_state}
  → trajectory_executor @20 Hz (1 s prestream, Offboard, ARM)  ← sole /fmu/in/* writer
  → flight_supervisor           (mission state machine + safety)
  → /drone/navigation/{state, landed, executor_state, px4_command_ack}
```

External data path: `PegasusSimulator (Isaac) ↔ MAVLink TCP 4560 ↔ PX4 SITL
(Docker) ↔ MicroXRCEAgent UDP 8888 ↔ host Fast DDS domain 45 ↔ ROS 2`. See
`src/px4_sitl_usage.md` and `docs/setup/isaacsim_scene_daemon.md`.

### Code-as-spec

`src/bridge_competition_pkg/bridge_competition_pkg/interface_audit.py::DEFAULT_REQUIRED_TOPICS`
is the **code-as-spec** for every topic/action/frame name. The runnable mirror
is `drone_interface_audit` (writes a JSON report to `report_path` and publishes
on `/drone/navigation/interface_audit`). The mutually exclusive low-speed
smoke-test entry point is `direct_rotor_smoke_test`. When you add/rename a
`/fmu/*`, `/drone/*`, or other UAV topic, update `DEFAULT_REQUIRED_TOPICS` and
`docs/contracts/interface_contracts.md` together.

### Ground state → air mission handoff

`competition_orchestrator_pkg` reads `/arena/ground/state` for one of
`{COMPLETE, SUCCESS, GROUND_DONE}`, then publishes `mission_request=true` to
`/drone/navigation/mission_request`. The supervisor state machine only leaves
`IDLE` when **both** `mission_request` and `ground_task_complete` are true. The
helper `ground_state_sim` exists to publish the ground signal during smoke
testing when the real ground task (`dual_arm_pkg` + `perception_competition_pkg`)
is not yet wired.

## Key Directories

- `src/grasp_demo_pkg/grasp_demo_pkg/` — YOLOE detector, depth pose, TF
  transform, plan-to-pose, gripper, pick/place state machine, shared `common.py`.
- `src/grasp_demo_pkg/launch/` — eight demo launches (basic arm, observation,
  perception, perception pipeline, pose TF, planning, gripper).
- `src/grasp_demo_pkg/config/demo_params.yaml` — single source of truth for
  grasp topic/frame names.
- `src/grasp_demo_pkg/weights/` — YOLOE + MobileCLIP weights, git-ignored.
- `src/drone_navigation_pkg/src/` — `flight_core.cpp` plus four `*_node.cpp`
  executables (`px4_state_adapter`, `ego_local_planner`, `trajectory_executor`,
  `flight_supervisor`).
- `src/drone_navigation_pkg/include/drone_navigation_pkg/flight_core.hpp` —
  public C++ surface: `Vec3`, `Quaternion`, `RollingVoxelMap`, `VoxelPlanner`,
  `UniformBsplineTrajectory`, `FlightSupervisor`, `FlightPhase` enum.
- `src/drone_navigation_pkg/msg/` — `Trajectory.msg`, `TrajectoryPoint.msg`.
- `src/drone_navigation_pkg/config/navigation.yaml` — `mission_autostart=false`
  by default, `px4_map_origin`, `drop_search_pose`. Do not flip without an
  explicit go-ahead and the calibrated values.
- `src/drone_navigation_pkg/THIRD_PARTY.md` — EGO-Planner upstream status.
- `src/bridge_competition_pkg/bridge_competition_pkg/interface_audit.py` —
  contract code-as-spec.
- `src/bridge_competition_pkg/bridge_competition_pkg/interface_contract.py` —
  shared contract dataclasses (pure helpers, no ROS).
- `src/bridge_competition_pkg/bridge_competition_pkg/rotor_smoke_test.py` —
  direct-rotor smoke test (mutually exclusive with PX4 mode).
- `src/bridge_competition_pkg/bridge_competition_pkg/chain_status.py` —
  single-page live status of the chain (replaces manual `ros2 topic echo`).
- `src/bridge_competition_pkg/bridge_competition_pkg/mission_trigger.py` —
  one-shot `/drone/navigation/mission_request=true` publisher.
- `src/bridge_competition_pkg/bridge_competition_pkg/ground_state_sim.py` —
  one-shot `/arena/ground/state=COMPLETE` publisher for smoke tests.
- `src/bridge_competition_pkg/setup.py` — console scripts `drone_interface_audit`,
  `direct_rotor_smoke_test`, `foxglove_daemon`, `chain_status`, `mission_trigger`,
  `ground_state_sim`.
- `src/competition_orchestrator_pkg/competition_orchestrator_pkg/mission_gate.py` —
  orchestrator state machine; `cargo_contract.py` defines cargo door contracts.
- `src/perception_competition_pkg/perception_competition_pkg/drop_target_detection.py` —
  reusable perception helpers; `drone_target_detector_node.py` is the node.
- `src/nav2_demo_pkg/launch/`, `config/`, `maps/`, `rviz/`, `scripts/`.
- `src/px4_sitl_usage.md` — PX4 v1.16.2 Docker external-simulator usage, DDS
  verification, ULog location.
- `docs/setup/` — env setup, EGO-Planner integration decision, Foxglove setup,
  Isaac Sim scene daemon.
- `docs/runbooks/` — drone navigation ops manual (6-stage rehearsal,
  calibration, acceptance), M1 modeling runbook.
- `docs/contracts/interface_contracts.md` — human mirror of
  `interface_audit.py`.
- `docs/project/tasks_full.md`, `docs/project/submission_checklist.md` —
  task tracking and pre-selection deliverables (deadline 2026-07-25 23:59).
- `.gitignore` — ignores `build/`, `install/`, `log/`, `__pycache__/`,
  `grasp_demo_debug/`, `**/weights/`, `*.pt`/`*.onnx`/`*.pth`/`*.h5`/`*.pkl`,
  `*.bag`, `*.log`, IDE/editor state.

## Development Commands

```bash
# One-time shell setup (every shell)
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash

# Build
colcon build
colcon build --packages-select isaac_ros2_control
colcon build --packages-select grasp_demo_interfaces grasp_demo_pkg
colcon build --packages-up-to grasp_demo_pkg   # rebuild a pkg + its deps
source install/setup.bash

# Tests
colcon test --packages-select drone_navigation_pkg perception_competition_pkg \
                          competition_orchestrator_pkg bridge_competition_pkg
colcon test-result --verbose

# Single Python test
python3 -m pytest src/<pkg>/test/test_<module>.py::TestCase::test_method

# Quick syntax check (NOT a ROS integration test)
python3 -m py_compile src/<pkg>/<pkg>/<node>.py

# Runtime contract check
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p report_path:=/tmp/drone_interface_report.json

# Single-page live chain status (use this instead of manual ros2 topic echo)
ros2 run bridge_competition_pkg chain_status

# Mission + ground state helpers (smoke test)
ros2 run bridge_competition_pkg mission_trigger
ros2 run bridge_competition_pkg ground_state_sim --state COMPLETE

# Direct-rotor smoke test (mutually exclusive with PX4; only for low-speed checks)
ros2 run bridge_competition_pkg direct_rotor_smoke_test \
  --ros-args -p enabled:=true -p rotor_speed_rad_s:=60.0

# Smallest Isaac Sim transport checks
ros2 run isaac_ros2_control cmd_vel_relay
ros2 run isaac_ros2_control tf_echo_bridge
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear:{x:0.5},angular:{z:0.3}}" -r 10

# Grasp demos
ros2 launch grasp_demo_pkg basic_arm_motion_demo.launch.py
ros2 launch grasp_demo_pkg perception_pipeline_demo.launch.py
ros2 launch grasp_demo_pkg planning_demo.launch.py

# Nav2 demos (most accept use_sim_time, map, autostart)
ros2 launch nav2_demo_pkg nav2_navigation.launch.py \
  map:=/path/to/map.yaml use_sim_time:=true
python3 src/nav2_demo_pkg/scripts/send_goal.py --x 2.0 --y 3.0 --yaw 1.57
python3 src/nav2_demo_pkg/scripts/save_map.py --map-name my_map

# Drone chain
ros2 launch drone_navigation_pkg navigation.launch.py
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py record_bag:=true

# Dual-mode sim: real Pegasus scene vs SIH host-only validation
sudo systemctl is-active isaacsim51-scene.service
cd /var/workspace/docker/isaac/docker/px4
docker compose down  # stop PX4 (any model)
PX4_SIM_MODEL=sihsim_quadx docker compose up -d  # host-only validation
docker compose down && docker compose up -d  # back to default gazebo-classic_iris
```

Do **not** add per-script `export ROS_DOMAIN_ID=…` lines.
`isaacsim51.service` already sets the canonical values
(`ROS_DOMAIN_ID=45`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DISTRO=jazzy`,
plus the Isaac ROS bridge `LD_LIBRARY_PATH`). Keep Isaac Sim, host, and the
PX4 Docker environment consistent on those four.

## Code Conventions & Common Patterns

- **Languages**: C++17 (only `drone_navigation_pkg` + vendored PX4) and
  Python 3 (every other package). New packages: `ros2 pkg create
  --build-type ament_python my_pkg --dependencies rclpy geometry_msgs`.
- **Node layout (Python)**: `src/<pkg>/<pkg>/<node>.py`; entry points declared
  in `setup.py` under `console_scripts`; `find_packages(exclude=['test'])`
  excludes the test tree from install; `data_files` installs `launch/`,
  `config/`, plus the ament index marker.
- **Node layout (C++)**: `src/<pkg>/src/<node>.cpp`; one `ament_add_gtest` per
  package when `BUILD_TESTING` is on; `flight_core` is a shared `add_library`
  linked by every node. Compile flags: `-Wall -Wextra -Wpedantic`, C++17.
- **Geometry types in C++**: use the `drone_navigation::Vec3` / `Quaternion`
  from `flight_core.hpp` inside the planner. External node boundaries use ROS
  messages (`geometry_msgs`, `nav_msgs`, `px4_msgs`).
- **Frame and convention discipline**: PX4 is **NED/FRD**; ROS topics are
  **ENU/FLU**; conversion is centralized in `px4NedFrdToRosEnuFlu` /
  `enuToNed` / `yawEnuToNed`. The map→world origin lives in `px4_map_origin`
  and is applied by `px4_state_adapter`. The planner uses `map` frame directly;
  the executor subtracts `px4_map_origin` when writing back to PX4 local.
- **Sole writer rule**: only `trajectory_executor` may publish `/fmu/in/*`.
  Direct rotor topics are reserved for `rotor_smoke_test` and are mutually
  exclusive with PX4 mode.
- **PX4 topic naming**: PX4 v1.16.2 with `px4_msgs` main branch uses
  `MESSAGE_VERSION` from each `.msg`. Topics with `MESSAGE_VERSION=1` are
  published with a `_v1` suffix (`vehicle_status_v1`). C++ subscribers hard-code
  the versioned name (`/fmu/out/vehicle_status_v1`); the audit's
  `resolve_actual_topic` auto-maps the unversioned contract name to the
  versioned live topic.
- **State machines**: drive them as enum classes + a small
  `update(inputs) → Decision` function. See `FlightSupervisor::update` and
  `MissionGate.observe` in `competition_orchestrator_pkg`.
- **Naming**: package names use `_pkg` suffix for competition packages; ROS
  nodes use `_node` suffix; actions end in `Command` (`DroneFlightCommand`,
  `CargoDoorCommand`); demo launch files end in `*_demo.launch.py`.
- **Config-driven behavior**: do **not** embed topic/frame names in node code
  when they could live in `config/<pkg>.yaml` or `demo_params.yaml`. New topics
  go through `interface_audit.py::DEFAULT_REQUIRED_TOPICS` first.
- **Build before use**: launch files, configs, and maps are installed under
  `install/`; rebuild and re-source `install/setup.bash` after editing any.
- **Action definition changes** require rebuilding `grasp_demo_interfaces`
  before rebuilding any server or client.
- **Dependency pins** (Python competition packages): `numpy<2`,
  `opencv-python>=4.6,<4.10`, `ultralytics`; install via package `setup.py`.
- **No `set -e`-style traps or per-script env exports** in launch/shell
  scripts — env comes from `isaacsim51.service`.
- **No `set -e`-style startup gates** — node bringup uses ROS lifecycle +
  parameter-driven enable flags (e.g. `mission_autostart=false`,
  `allow_manual_*_actions=false`), not shell traps.

## Important Files

- `CLAUDE.md` — engineer onboarding (build/test commands, package boundaries,
  cross-package contracts).
- `AGENTS.md` — this file.
- `src/README.md` — package index, drone stack status, workflow, env reminders.
- `src/px4_sitl_usage.md` — PX4 v1.16.2 Docker usage and `/fmu` topic contract.
- `src/grasp_demo_pkg/config/demo_params.yaml` — grasp topic/frame names.
- `src/grasp_demo_pkg/weights/` — YOLOE + MobileCLIP weights (git-ignored).
- `src/drone_navigation_pkg/CMakeLists.txt` — defines the `flight_core` lib +
  four executables + the gtest.
- `src/drone_navigation_pkg/include/drone_navigation_pkg/flight_core.hpp` —
  public C++ surface (planner, splines, supervisor).
- `src/drone_navigation_pkg/config/navigation.yaml` — `mission_autostart=false`,
  `px4_map_origin`, `drop_search_pose`.
- `src/drone_navigation_pkg/THIRD_PARTY.md` — EGO-Planner upstream status.
- `src/bridge_competition_pkg/bridge_competition_pkg/interface_audit.py` —
  **code-as-spec** for topic/action/frame names.
- `src/bridge_competition_pkg/bridge_competition_pkg/interface_contract.py` —
  shared contract dataclasses.
- `src/bridge_competition_pkg/bridge_competition_pkg/rotor_smoke_test.py` —
  mutually exclusive low-speed direct-rotor smoke test.
- `src/bridge_competition_pkg/bridge_competition_pkg/chain_status.py` —
  single-page live chain status (NEW: replaces manual `ros2 topic echo`).
- `src/bridge_competition_pkg/bridge_competition_pkg/mission_trigger.py` —
  one-shot mission-request publisher.
- `src/bridge_competition_pkg/bridge_competition_pkg/ground_state_sim.py` —
  one-shot ground-state publisher.
- `src/bridge_competition_pkg/setup.py` — installs entry points
  `drone_interface_audit`, `direct_rotor_smoke_test`, `foxglove_daemon`,
  `chain_status`, `mission_trigger`, `ground_state_sim`.
- `src/grasp_demo_interfaces/action/` — `DetectObject.action`,
  `PlanToPose.action`, `GripperCommand.action`, `DroneFlightCommand.action`,
  `CargoDoorCommand.action`.
- `src/drone_navigation_pkg/msg/Trajectory.msg`,
  `src/drone_navigation_pkg/msg/TrajectoryPoint.msg`.
- `docs/contracts/interface_contracts.md` — human mirror of
  `interface_audit.py`.
- `docs/runbooks/drone_navigation.md` — drone ops manual (6-stage rehearsal,
  calibration, acceptance; 10 consecutive no-crash runs, 90% drops within 0.2 m).
- `docs/setup/env_setup.md` — Pegasus + Isaac Sim 5.1 + cuRobo + ROS 2 Jazzy +
  PX4 v1.16.2 install plan.
- `docs/setup/ego_planner_integration.md` — EGO-Planner integration decision
  (upstream commit `23a8d5a19…` not vendored; raycast/LBFGS ticket open).
- `docs/setup/foxglove_setup.md` — Foxglove WebSocket bridge (port 8765,
  systemd --user, `foxglove-bridge.service` with `LINGER=yes`).
- `docs/setup/isaacsim_scene_daemon.md` — Isaac Sim 5.1 + Pegasus
  `PX4MavlinkBackend` ↔ MicroXRCEAgent wiring; dual-mode (SIH vs Pegasus)
  decision; VNC password `robo2026`.
- `docs/project/tasks_full.md`, `docs/project/submission_checklist.md` — M1–M9
  task tracking, submission deadline 2026-07-25 23:59, to
  `airobot@turingltd.com`.
- `.gitignore` — ignores `build/`, `install/`, `log/`, `__pycache__/`,
  `grasp_demo_debug/`, `**/weights/`, `*.pt`/`*.onnx`/`*.pth`/`*.h5`/`*.pkl`,
  IDE/editor state.

## Runtime / Tooling Preferences

- **ROS distribution**: Jazzy (only). Do not mix Humble/Iron/Foxy code paths.
- **Middleware**: `rmw_fastrtps_cpp` (Fast DDS). Domain ID **45** is fixed
  across Isaac Sim, host, and PX4 Docker.
- **Build system**: `colcon` over a single `src/` tree. C++ packages use
  `ament_cmake`; Python packages use `ament_python`. Custom messages/actions
  are generated via `rosidl_default_generators` /
  `rosidl_generate_interfaces`.
- **Required host tools**: `colcon`, `ros-jazzy-*` desktop, `python3-pytest`,
  `ament_cmake_gtest` (only needed for the drone pkg); `mypy_cache` /
  `pytest_cache` are git-ignored.
- **External simulators** (outside this repo):
  - Isaac Sim 5.1 (`conda isaacsim51` env) + PegasusSimulator on the host.
  - PX4 v1.16.2 SITL in Docker at `/var/workspace/docker/isaac/docker/px4/`,
    external-simulator mode, PegasusSimulator TCP **4560** → uXRCE-DDS UDP
    **8888** → Fast DDS domain **45**.
  - Isaac assets deployed at `/var/workspace/docker/isaac/scenes/`
    (`X1_assets/`, `bobac_assets/`); the `.zip` source lives at
    `/var/workspace/docker/isaac/incoming/*.zip` and is git-ignored.
- **PX4 simulation backends** (mutually exclusive; the active one is set by
  `PX4_SIM_MODEL` in `docker-compose.yml`):
  - `gazebo-classic_iris` (default): expects Pegasus to drive physics. Requires
    `isaacsim51-scene.service` active and the scene `Play`ed via VNC.
  - `sihsim_quadx`: PX4's software-in-the-loop physics — no Pegasus required.
    Used for host-chain validation when Pegasus's `is_homogeneous` race prevents
    proper physics initialization. Switch with
    `cd /var/workspace/docker/isaac/docker/px4 && docker compose down && PX4_SIM_MODEL=sihsim_quadx docker compose up -d`.
- **Foxglove**: WebSocket bridge on port **8765**, managed by `systemd --user`
  with `LINGER=yes`. Use the `foxglove_daemon` entry point; the
  `bridge_competition_pkg` launch and `scripts/foxglove_daemon.sh` are the
  canonical entry points. The tmux fallback is deprecated.
- **Bags / logs**: `bags/` and `log/` at the workspace root are for ad-hoc
  rosbag captures and `colcon` logs; both are ignored. PX4 ULog + rosbag
  capture persists under `docker/px4/{logs,bags}/`.
- **Editor**: no repo-pinned formatter, lint config, or pre-commit hook
  exists yet. When adding one, match the project style (PEP 8 + isort-shaped
  imports; C++ uses Google-style or ROS-style braces; do not introduce
  competing conventions).
- **No CI** in `.github/`, `.gitlab-ci*`, or `pre-commit` config is checked
  in. Tests run locally via `colcon test`.

## Testing & QA

- **Two test styles, no coverage tooling**:
  - **C++ gtest** (`drone_navigation_pkg`): `test/test_flight_core.cpp` covers
    `CoordinateFrames`, `VoxelPlanner` detour, `RollingVoxelMap` retention,
    `UniformBsplineTrajectory` endpoint + dynamic-limit sampling,
    `FlightSupervisor` safety gates and phase sequence, and
    `executorSafetyAction` watchdog. Nine `TEST(...)` cases, no gmock — pure
    assertions on returned structs.
  - **Python unittest** (`bridge_competition_pkg`,
    `perception_competition_pkg`, `competition_orchestrator_pkg`): test files
    live in `src/<pkg>/test/`, use `unittest.TestCase` (no pytest fixtures,
    no `conftest.py`, no mocks). The perception tests synthesize images with
    `cv2.circle`/`cv2.rectangle`.
- **Test registration**:
  - C++: `if(BUILD_TESTING) find_package(ament_cmake_gtest); ament_add_gtest(...);
    target_link_libraries(test_flight_core flight_core)` (see
    `src/drone_navigation_pkg/CMakeLists.txt`).
  - Python: `setup.py` declares `test_suite='test'` and
    `packages=find_packages(exclude=['test'])`; `package.xml` adds
    `ament_pep257`, `ament_flake8`, `python3-pytest` as `test_depend`.
- **Standard runner**:
  ```bash
  colcon test --packages-select drone_navigation_pkg \
                          perception_competition_pkg \
                          competition_orchestrator_pkg \
                          bridge_competition_pkg
  colcon test-result --verbose
  ```
- **Single test** (fully qualified, including the test class):
  ```bash
  python3 -m pytest \
    src/bridge_competition_pkg/test/test_interface_contract.py::InterfaceContractTest::test_<method>
  ```
- **Lint-only packages**: `nav2_demo_pkg` and `px4_msgs`
  only run `ament_lint_auto_find_test_dependencies()` (lint, no unit tests).
- **`dual_arm_pkg`** has no tests yet — `package.xml` declares the
  `test_depend`s but `test/` is empty.
- **`grasp_demo_pkg`, `grasp_demo_interfaces`, `isaac_ros2_control`** have no
  test suite; only `package.xml` test_depend declarations.
- **Coverage**: no coverage tool configured (no `coverage.py`, no
  `gcovr`). Treat each new test as a behavioral contract for a
  planner/safety/contract decision, not plumbing.
- **Runtime contract QA**: `ros2 run bridge_competition_pkg drone_interface_audit`
  is the runnable mirror of `interface_audit.py::DEFAULT_REQUIRED_TOPICS`.
  It periodically checks the live required-topic graph, publisher/subscriber
  connectivity, FMU-input writer ownership, resolved PX4 topic names (auto
  resolving `_v1` suffixes), QoS endpoint data, and observed frequency/frame
  metadata; the `ok` field in its JSON summary is the runtime gate. Treat its
  JSON report as the gating check before/after any topic rename.
- **Single-page live status**: `ros2 run bridge_competition_pkg chain_status`
  prints the audit summary plus the latest `/drone/navigation/{px4_status,
  state, executor_state, planner_state, landed}` values. Use this from VNC
  before opening Foxglove to answer "did the chain come up?".
- **Smoke-test helpers**: `mission_trigger` and `ground_state_sim` exist to
  drive the supervisor out of `IDLE` when the ground task stack
  (`dual_arm_pkg` + perception) is not yet wired. They are not part of the
  acceptance path — they are tooling for host-side M9.x smoke tests.
- **Manual flight rehearsal**: `docs/runbooks/drone_navigation.md` describes a
  6-stage rehearsal; `mission_autostart=false` is the default and must stay
  false until the calibration ticket is closed. Acceptance: 10 consecutive
  no-crash runs, 90% drops within 0.2 m of target.
