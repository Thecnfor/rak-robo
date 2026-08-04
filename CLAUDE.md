# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace and environment

This repository is a ROS 2 Jazzy `colcon` workspace. The repository root is `/var/workspace/docker/isaac/workspace`; all ROS packages live directly under `src/`. `build/`, `install/`, and `log/` are generated workspace output and are ignored by Git.

The Isaac Sim service configures the ROS 2 transport environment: `ROS_DOMAIN_ID=45`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DISTRO=jazzy`, and the Isaac Sim ROS bridge library path. Keep these values consistent between the Isaac Sim side, the host, and the PX4 Docker environment, but do not add per-script `export` commands; `src/README.md`, `docs/README.md` (index), `docs/setup/env_setup.md`, and `src/px4_sitl_usage.md` document the environment setup.

PX4 SITL is the selected drone flight controller and runs from `/var/workspace/docker/isaac/docker/px4/`. EGO-Planner is not deployed as an upstream ROS 2 stack: its isolated planning core, ROS adapters, supervisor, and the unique PX4 Offboard writer live in `drone_navigation_pkg`. Do not publish `/fmu/in/*` outside `trajectory_executor`; direct rotor topics are only for the mutually exclusive low-speed smoke-test mode.

The normal shell setup is:

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
```

## Local development vs Socl

Code is developed locally and synced to Socl through GitHub (the only
interchange). The two environments differ and are not interchangeable:

- **Socl** (`/var/workspace/docker/isaac/workspace`): ROS 2 Jazzy,
  Ubuntu 24.04, Python 3.12. Authoritative build, PX4 SITL, Isaac Sim
  5.1, real-machine verification. `ROS_DOMAIN_ID=45` is the live
  competition domain.
- **Local** (`~/Desktop/ROBOTAC/rak-robo`): ROS 2 Lyrical, Ubuntu 26.04,
  Python 3.14. Code editing and logic regression only. Do **not** set
  `ROS_DOMAIN_ID` locally — there are no DDS peers, so everything fails
  closed and cannot disturb the live stack.

Local build/test caveats and the full dev loop live in
`docs/runbooks/local_dev.md`. Key points: the interactive shell is zsh,
so source ROS inside bash
(`bash -c 'source /opt/ros/lyrical/setup.bash && ...'`); locally skip
`nav2_demo_pkg` (Nav2 is not yet released for Lyrical) and `px4_ros_com`
(uses a CMake API removed in Lyrical).

## Build, test, and lint commands

Build the whole workspace, or select a package when iterating:

```bash
colcon build
colcon build --packages-select isaac_ros2_control
colcon build --packages-select grasp_demo_interfaces grasp_demo_pkg
source install/setup.bash
```

`grasp_demo_pkg` imports generated actions from `grasp_demo_interfaces`, so build the interface package whenever action definitions change. `colcon build --packages-up-to grasp_demo_pkg` is also useful when rebuilding a package and its dependencies.

The UAV packages have checked-in tests: `drone_navigation_pkg` uses gtest for frames, planning, B-spline feasibility, and supervisor safety; perception, orchestrator, and bridge use Python unit tests discovered through their `setup.py` test suites.

```bash
colcon test --packages-select drone_navigation_pkg perception_competition_pkg competition_orchestrator_pkg bridge_competition_pkg dual_arm_pkg
colcon test-result --verbose
```

To run one Python test directly, use its unittest module or pytest selector, for example:

```bash
python3 -m pytest src/<package>/test/test_<module>.py::test_<name>
```

For a quick syntax check of one ROS Python node or helper script (not a substitute for a ROS integration test):

```bash
python3 -m py_compile src/grasp_demo_pkg/grasp_demo_pkg/<node>.py
python3 -m py_compile src/nav2_demo_pkg/scripts/<script>.py
```

After sourcing `install/setup.bash`, the runtime contract check is `drone_interface_audit` (installed by `bridge_competition_pkg`). It writes a JSON report at the configured path and is the runnable mirror of `bridge_competition_pkg/interface_audit.py::DEFAULT_REQUIRED_TOPICS`. The mutually exclusive direct-rotor smoke test entry point is `direct_rotor_smoke_test`:

```bash
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p report_path:=/tmp/drone_interface_report.json
ros2 run bridge_competition_pkg direct_rotor_smoke_test \
  --ros-args -p enabled:=true -p rotor_speed_rad_s:=60.0
```

## Package build conventions

- **C++ targets**: link dependencies with
  `target_link_libraries(tgt ${pkg}_TARGETS)` (imported targets). Do
  **not** use `ament_target_dependencies()` — it was removed in ROS 2
  Lyrical; `drone_navigation_pkg` has already been migrated. The
  imported-target idiom works on both Jazzy and Lyrical.
- **ament_python tests**: `colcon test` discovers tests via
  `test_suite='test'` in `setup.py` plus an empty `test/__init__.py`.
  See `bridge_competition_pkg` and `dual_arm_pkg` for the pattern.
  `grasp_demo_pkg` and `isaac_ros2_control` have no test suite.
- **Two test tiers**: instant (no build, no source) runs pure-Python
  logic in <1 s — `test_hover_probe_core`, `test_dynamic_obstacle_probe`,
  `test_interface_contract`, `test_stage2_acceptance`, `test_pick_place`.
  Full (`colcon test`) adds the 62-case `test_flight_core` gtest
  (coordinate frames, return/descent latch, voxel planner, B-spline
  feasibility, supervisor state machine) plus the remaining packages.
- **New interfaces** (`.msg`/`.action`/`.srv`): rebuild
  `grasp_demo_interfaces` first, then dependents. Current examples:
  `DualGripperCommand.action` and `Trajectory.msg`'s `preemption_reason`.

## Running the demos

After building and sourcing `install/setup.bash`, the smallest Isaac Sim transport checks are:

```bash
ros2 run isaac_ros2_control cmd_vel_relay
ros2 run isaac_ros2_control tf_echo_bridge
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear:{x:0.5},angular:{z:0.3}}" -r 10
```

`cmd_vel_relay` and `tf_echo_bridge` are observability stubs. Isaac Sim's OmniGraph is the component that actually consumes `/cmd_vel` and publishes `/tf`; the control package does not drive actuators itself. The Isaac-side setup (ROS2Context with domain 45, Twist subscriber, PublishTF, and optionally `/clock`) is described in `src/isaac_ros2_control/README.md`.

Useful grasp launches are:

```bash
ros2 launch grasp_demo_pkg basic_arm_motion_demo.launch.py
ros2 launch grasp_demo_pkg observation_pose_demo.launch.py
ros2 launch grasp_demo_pkg perception_demo.launch.py
ros2 launch grasp_demo_pkg perception_pipeline_demo.launch.py
ros2 launch grasp_demo_pkg pose_tf_demo.launch.py
ros2 launch grasp_demo_pkg planning_demo.launch.py
ros2 launch grasp_demo_pkg gripper_demo.launch.py
```

The Nav2 demos are:

```bash
ros2 launch nav2_demo_pkg slam_toolbox_online.launch.py
ros2 launch nav2_demo_pkg amcl_localization.launch.py
ros2 launch nav2_demo_pkg nav2_navigation.launch.py
ros2 launch nav2_demo_pkg costmap_only.launch.py
ros2 launch nav2_demo_pkg planner_test.launch.py
ros2 launch nav2_demo_pkg controller_test.launch.py
```

Most Nav2 launches accept `use_sim_time`, `map`, and `autostart` arguments. For example:

```bash
ros2 launch nav2_demo_pkg nav2_navigation.launch.py map:=/path/to/map.yaml use_sim_time:=true
python3 src/nav2_demo_pkg/scripts/send_goal.py --x 2.0 --y 3.0 --yaw 1.57
python3 src/nav2_demo_pkg/scripts/save_map.py --map-name my_map
```

The drone chain launches are separate from the demo packages. `drone_navigation_pkg/navigation.launch.py` brings up only the four-node PX4 control chain; `bridge_competition_pkg/host_bridge_bringup.launch.py` aggregates the chain with perception, the orchestrator, and `drone_interface_audit` (and accepts `record_bag:=true` for an opt-in rosbag). Both default to `mission_autostart=false` in `drone_navigation_pkg/config/navigation.yaml`; do not flip this without an explicit go-ahead and the calibrated `drop_search_pose` and `px4_map_origin`:

```bash
ros2 launch drone_navigation_pkg navigation.launch.py
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py record_bag:=true
```

## Architecture

### Package boundaries

- **`dual_arm_pkg`** owns the competition dual-arm, dual-gripper, and ground-task state machine implementation.
- **`perception_competition_pkg`** owns competition perception, depth pose estimation, and drone downward-camera alignment.
- **`bridge_competition_pkg`** owns host bringup, runtime interface audit, Foxglove, and the explicitly armed direct-rotor smoke test. It does not own planning or publish PX4 setpoints.
- **`drone_navigation_pkg`** owns PX4 NED/FRD state adaptation, the isolated EGO-style voxel/A*/B-spline core, local planning, the unique 20 Hz Offboard executor, and the flight safety supervisor.
- **`competition_orchestrator_pkg`** owns the air-ground coordinator, cargo-door actions, and high-level drone mission actions. It sends goals through the planning/PX4 chain rather than commanding Pegasus rotors directly.
- **`grasp_demo_interfaces`** is an `ament_cmake` interface package. It also generates `DroneFlightCommand` and `CargoDoorCommand`; contract changes require rebuilding the interface package and both servers/clients.
- **`grasp_demo_pkg`** is an `ament_python` package containing the perception, geometry, arm-motion, gripper, and pick/place teaching nodes. Its `setup.py` installs the Python console entry points plus launch files, YAML parameters, and model weights.
- **`nav2_demo_pkg`** is an `ament_cmake` package that installs Nav2 launch files, YAML configuration, maps, RViz layouts, and helper scripts. It launches nodes from the installed Nav2 stack rather than implementing a navigation stack in this repository.
- **`isaac_ros2_control`** is an independent minimal `ament_python` starter for verifying host-to-Isaac and Isaac-to-host ROS topic connectivity.
- **`px4_msgs`** and **`px4_ros_com`** provide the PX4 ROS 2 contracts, Offboard examples, and frame transforms. Keep message definitions synchronized with the PX4 SITL version.

### Grasp perception-to-planning flow

The canonical wiring is configured in `src/grasp_demo_pkg/config/demo_params.yaml` and composed by `perception_pipeline_demo.launch.py`:

```text
/arm_camera/rgb
  -> yoloe_detector_node
  -> /demo_grasp/object_mask, /demo_grasp/bbox, label, confidence, debug_image

/arm_camera/depth + /arm_camera/camera_info + bbox/mask
  -> depth_pose_estimator_node
  -> /demo_grasp/object_point_camera, /demo_grasp/long_axis_camera

camera-frame point/axis + TF
  -> tf_transform_demo_node
  -> /demo_grasp/object_point_base, /demo_grasp/normal_base,
     /demo_grasp/long_axis_base

pick_place_state_machine
  -> DetectObject action (/demo_detect_object)
  -> PlanToPose action (/demo_plan_to_pose) for candidate pre-grasp,
     grasp, lift, pre-place, and place poses
```

`yoloe_detector_node` loads `weights/yoloe-26l-seg.pt` (and prepares `mobileclip2_b.ts`), selects the best detection, and publishes the 2D results. It requires `ultralytics` and defaults to `cuda:0`; model paths are resolved relative to the installed package's `weights/` directory unless absolute.

`depth_pose_estimator_node` uses the median valid depth around the 2D bounding-box center to back-project a 3D point. It estimates the object's image-plane long axis using PCA over the mask and publishes a camera-frame vector. `tf_transform_demo_node` applies the available TF into `base_link_arm` and normally publishes a fixed downward normal.

`plan_to_pose_node` validates the point against the configured workspace, constructs a candidate `PoseStamped` using the normal and long axis, and publishes it. Its optional execution is deliberately a smooth, fixed-joint interpolation demo on `/hand_command`, not IK, MoveIt, or a real trajectory planner. `gripper_demo_node` exposes `/demo_gripper_command` and publishes the mapped gripper `JointState` command on `/hand_command`.

Despite its name, `pick_place_state_machine` currently generates and logs candidate grasp/place poses with `execute=False`; its run path does not perform real grasp, lift, or placement execution. Treat changes that turn this into physical execution as a separate safety-sensitive behavior change.

### Navigation flow

Nav2 configuration assumes simulation time and the robot frame chain `map -> odom -> base_link`. The AMCL and costmap configs consume `/laser_scan_fuse`; the robot/Isaac side must provide the expected odometry, TF, laser, and clock topics. `nav2_navigation.launch.py` starts map server and AMCL immediately, then starts planner/controller/behavior/BT servers after a delay and manages them through lifecycle managers. The other launch files isolate SLAM, localization, costmaps, planner testing, or controller testing and pair them with package-provided RViz layouts.

`goal_to_plan.py` listens for RViz `/goal_pose`, calls Nav2's `ComputePathToPose`, and republishes the returned path on `/plan`. `send_goal.py` sends a `NavigateToPose` action goal from command-line coordinates. `amcl_nomotion_update.py` publishes an initial pose and requests AMCL no-motion updates; `save_map.py` wraps `nav2_map_server`'s `map_saver_cli`.

### Drone chain architecture

`drone_navigation_pkg/src/` is a single C++ (`ament_cmake`) library `flight_core` plus four nodes wired in series by `launch/navigation.launch.py` and configured by `config/navigation.yaml`:

```text
/fmu/out/*  (PX4 SITL, BEST_EFFORT QoS)
  -> px4_state_adapter   (NED/FRD -> map ENU/FLU, applies px4_map_origin)
  -> /drone/navigation/odometry  + /drone/navigation/px4_status
  -> ego_local_planner          (rolling voxel / dynamic A* / uniform B-spline)
  -> /drone/navigation/{trajectory,planned_path,planner_state}
  -> trajectory_executor @20 Hz (unique /fmu/in/* writer, 1 s prestream, Offboard, ARM)
  -> flight_supervisor           (mission state machine + safety)
  -> /drone/navigation/{state,landed,executor_state,px4_command_ack}
```

`ego_local_planner` is an isolated ESDF-free voxel/A\*/B-spline core behind the `VoxelPlanner` / `UniformBsplineTrajectory` interfaces in `include/drone_navigation_pkg/flight_core.hpp`; the requested upstream EGO-Planner commit (`23a8d5a19…`) is **not** vendored and the LBFGS/raycast ticket is still open per `src/drone_navigation_pkg/THIRD_PARTY.md`. `bridge_competition_pkg` deliberately does **not** plan or publish PX4 setpoints — it only transports, audits, and (under `DRONE_BACKEND=direct_rotor`) runs the per-rotor smoke test. Full topic/action contract and runtime audit protocol live in `docs/contracts/interface_contracts.md` and `docs/runbooks/drone_navigation.md`.

## Cross-package contracts to preserve

Topic names, action names, frame names, and parameter keys are the integration surface of these demos. Prefer changing `src/grasp_demo_pkg/config/demo_params.yaml` and the corresponding launch/client/server together rather than embedding a new value in one node. In particular, grasp code expects `/hand_command`, `arm_Camera`, and `base_link_arm`, while Nav2 uses `/laser_scan_fuse`, `map`, `odom`, and `base_link`.

For the drone chain, the **code-as-spec** is `bridge_competition_pkg/bridge_competition_pkg/interface_audit.py::DEFAULT_REQUIRED_TOPICS`: when you add, remove, or rename a `/fmu/*`, `/drone/*`, or other UAV topic, update that list, then update `docs/contracts/interface_contracts.md` to match. Renaming or adding an action (`/drone/flight_command`, `/cargo_bay/door_command`) means rebuilding `grasp_demo_interfaces` first, then both servers and clients.

When modifying an action definition, rebuild `grasp_demo_interfaces` before rebuilding/running `grasp_demo_pkg`. When modifying launch/config/map/RViz assets, remember that the package build installs copies under `install/`; rebuild and re-source before testing the installed launch file. Generated build artifacts, Python caches, ROS bags/logs, and `grasp_demo_debug/` output are intentionally ignored; the deployed Isaac assets under `/var/workspace/docker/isaac/scenes/` are outside this repository.
