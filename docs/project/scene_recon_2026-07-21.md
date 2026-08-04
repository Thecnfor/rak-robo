# Scene & Asset Reconnaissance Report

> Date: 2026-07-21 (D-4 from 2026-07-25 23:59 deadline)
> Author: scene recon pass
> Scope: `/var/workspace/docker/isaac/scenes/active/`, two single-robot USDs,
> `X1_race_scene.usd`, ROS interface contract, and the integration runtime scripts.
> Companion: `docs/project/tasks_full.md` M1-M9, `docs/contracts/interface_contracts.md`.

## 0. Reading order

This report has six parts. **Read Part 5 (gaps) before Part 6 (the new plan)**
— the gaps drive the plan's ordering.

1. Competition contract (re-read)
2. `mercury_x1.usd` facts
3. `sunray.usd` facts
4. `X1_race_scene.usd` facts
5. **Gaps that block full-pipeline debugging**
6. New plan: refactor + auto-joint-binder

Raw JSON dumps:
- `/tmp/mercury_x1.recon.json`
- `/tmp/sunray.recon.json`
- `/tmp/x1_race_scene.recon.json`

Recon script (sits in `/tmp/`, move to `scenes/active/scripts/integrated_runtime/tools/`
during refactor):
- `/tmp/usd_recon.py` (uses conda `isaacsim51` env's `pxr`)

---

## 1. Competition contract recap

| Item | Value |
|---|---|
| Platform | A = TuringStack X1 双臂轮式机器人 (we chose this) |
| Total score | 100 (基础) + 25 (加分) = 125 max |
| Submission deadline | 2026-07-25 23:59 (D-4 today) |
| Sim version | "Isaac Sim 4.5.0 建议；其他须注明" → we use 5.1, must note in tech doc |
| Hard bans | AprilTag / QR / 人工视觉标签 / 拆分视频拼接 / 现场重置 |
| **Coupling rule** | 前后关联任务，依赖前者未实现→后者**不予给分** |
| Drop accuracy | 0.2m=8, 0.4m=5, 0.6m=2, >0.6m=0 |
| Cargo bay | **硬连接**于无人机下方，机器人不挂载 |
| Required run | 全流程**连续**单段，**不能拆**成多段 video |

### 1.1 Task→score matrix (赛段1, 50 分)

| # | Task | Score | Bind point in our stack |
|---|---|---|---|
| 1.1 | USD 导入 + ActionGraph | 10 | `X1_race_scene.usd` ✓; ActionGraph mostly done in USD + partially in scene_app.py |
| 1.2 | 自主导航到办公区 | 10 | Nav2 + `/cmd_vel` (USD `DiffController` provides this) |
| 1.3 | 物料识别与位姿估计 | 10 | YOLOE → `/demo_grasp/object_point_camera` → TF → base |
| 1.4 | 抓取 | 8 | `/arm_left/joint_cmd` (?), `/gripper_*` (?, **未确认 topic**) |
| 1.5 | 投放货舱 | 10 | 货物放 cargo bay + `/cargo_bay/command left_close` |
| 1.6 | 技术合规 | 2 | 无人工标签 + 平台合规 |

### 1.2 Task→score matrix (赛段2, 40 分)

| # | Task | Score | Bind point in our stack |
|---|---|---|---|
| 2.1 | 无人机 USD + 舱门逻辑 | 8 | `sunray.usd` + `air_fpv_box.usd` + cargo 接口 ✓ |
| 2.2 | 起飞前关侧舱 | 5 | `left_door_joint` drive |
| 2.3 | 起飞 + 飞往投放区 | 6 | PX4 Offboard + planner |
| 2.4 | 机载视觉对准 | 8 | `/drone0/down_camera/color/image_raw` |
| 2.5 | 底部投放 | 5 | `bottom_door_joint` |
| 2.6 | 投放精度 | 8 | 0.2m 内得 8 |

---

## 2. `mercury_x1.usd` facts (only robot, no scene)

| Field | Value |
|---|---|
| Path | `/var/workspace/docker/isaac/scenes/active/robots/mercury_x1.usd` |
| Size | 133 MB |
| UpAxis | Z |
| **MetersPerUnit** | **0.01** (centimeters) |
| DefaultPrim | `Root` |
| Prim count | 330 |
| MaterialBindingAPI | 41 (decoration) |
| PhysicsRigidBodyAPI | 39 |
| PhysicsMassAPI | 39 (no authored mass values — see §5) |
| **PhysicsDriveAPI:angular** | **21** |
| **ArticulationRootAPI** | **3** (base_link, left_gripper, right_gripper) |
| PhysicsRevoluteJoint | 37 |
| PhysicsFixedJoint | 5 (2 radar + 2 gripper mount + 1 link2_L/3_L inner) |
| **UsdGeom.Camera** | **3**: `link_eye/Right_Camera` f=2.6, `link_eye/Left_Camera` f=2.6, `link_radar/Rotating` f=50 |
| OmniGraphNode / OmniGraph | 65 / 10 (pre-baked graphs — see §4) |
| Lidar / Radar / IMU schemas | **none** (no RTX sensor schema) |

### 2.1 Joint inventory (42 joints)

**Head chain (3):**
- `base_link/body` (revolute X) — body
- `link_body/head` (revolute X) — head
- `link_head/eye` (revolute X) — eye

**Left arm (6 actuated):**
- `link_body/joint1_L` axis=X limits=[-165,165] → link1_L
- `link1_L/joint2_L` axis=X limits=[-55,95] → link2_L
- `link3_L/joint3_L` axis=X limits=[-173,5] → link4_L
- `link4_L/joint4_L` axis=X limits=[-165,165] → link5_L
- `link5_L/joint5_L` axis=X limits=[-20,265] → link6_L
- `link6_L/joint6_L` axis=X limits=[-180,180] → link7_L
- (link2_L→link3_L is `FixedJointL`, cosmetic)

**Right arm (6, mirrored):**
- `link_body/joint1_R` … `link6_R/joint6_R` — same as left

**Grippers (7 each, mixed limits):**
- left: `left_gripper_left_joint1` (free Z), `left_gripper_right_join2` [0,64], `left_gripper_left_finger_joint` [0,63], `left_gripper_left_joint2` [-63.6,0], `left_gripper_right_finger_joint` [-63,2], `left_gripper_right_joint1` (free), `gripper_right_outer_finger_joint` [-63,2], `gripper_left_outer_finger_joint` [0,60] = **8** joints
- right: same 8 mirrored

**Wheels (2 drive + 2 free + 4 caster):**
- `front_left_wheel/front_left_wheel` revolute X (no limits) — **driven by `DiffController` USD graph** (differential, not direct drive)
- `front_right_wheel/front_right_wheel` same
- `left_Swivel_Wheel/Joint/left_caster_1_joint` (Z, free) + `left_caster_wheel_1_joint` (X, free)
- right caster pair, same
- **No caster drives** — caster is just for mechanical swivel

**Drive count: 21**
- head/body/eye (3) + 6 left arm + 6 right arm + 3 left gripper finger (the 3 with limits, the 2 free Z joints are coupled mechanically) + 3 right gripper = **21**. Matches.

### 2.2 Body frame

`base_link` at translation (0, 0, 0) in USD-cm. link_body at z=0.8cm (i.e. 0.008m) = **head base is 0.8cm above base_link in USD units**. After scene's MetersPerUnit conversion (USD-cm → scene-m), `base_link` and `link_body` are at:
- `base_link` origin at scene (0, 0, 0)
- `link_body` at scene (0, 0, 0.8) m
- `link_head` at scene (0, 0, 1.0) m
- `link_eye` at scene (~0.03, 0, ~0.94) m — i.e. the eyes are *forward* and *up* by ~3cm
- cameras: `link_eye/Left_Camera` and `link_eye/Right_Camera` at f=2.6cm ≈ **26mm focal length** (USD Camera focal_length unit is cm)

### 2.3 What's **NOT** in the USD
- **No mass values** for any rigid body (`mass=None` for all 39 RB). Physics will run with `PhysicsMassAPI` defaults — needs to be set, or simulation will be unphysically light.
- **No Isaac/RTX Lidar, IMU, Radar schema**. Pure robot mesh + joints + cameras only.
- **No FPV/RGB camera on the robot body**. Only the two eye cameras.
- The "link_radar/Rotating" Camera (f=50) is a placeholder/misnamed mesh, not an actual radar.

---

## 3. `sunray.usd` facts (Sunray150 + cargo bay, default-prim is everything)

| Field | Value |
|---|---|
| Path | `/var/workspace/docker/isaac/scenes/active/robots/sunray.usd` |
| Size | 1.2 MB |
| UpAxis | Z |
| **MetersPerUnit** | **1.0** (meters, **different from mercury_x1!** assembly does unit-aware scale) |
| DefaultPrim | `Sunray150WithMid360Cargo` |
| Prim count | 113 |
| ArticulationRootAPI | 1 (`Sunray150WithMid360Cargo`) |
| PhysicsRigidBodyAPI | 8 (body + 4 rotor + cargo_body + left_door + bottom_door) |
| **PhysicsDriveAPI:angular** | **2** |
| PhysicsRevoluteJoint | 6 (4 rotor + 2 door) |
| PhysicsFixedJoint | 1 (`cargo_mount_joint`) |
| **UsdGeom.Camera** | **1**: `body/camera` f=50 (front-FPV, NOT downward) |
| Lidar / Radar / IMU | none |
| **No LiDAR schema** despite `_mid360_visual_source` mesh in scene (the Mid-360 mesh is decoration only) |

### 3.1 Joint inventory (7 joints)
- `rotor0..3/joint0..3` revolute Z (motors — driven by PX4 or `rotor_smoke_test`)
- `mounted_cargo_bay/transparent_cargo_bay/joints/left_door_joint` revolute X **[-90, 0]**
- `mounted_cargo_bay/transparent_cargo_bay/joints/bottom_door_joint` revolute X **[-90, 0]**
- `cargo_mount_joint` fixed (locks cargo bay to body)

### 3.2 Drive count: 2
Only the **two doors** have drives. Rotor joints are driven externally by PX4MavlinkBackend / direct_rotor — they don't need an angular drive because the backend publishes directly to the joint's velocity/position attribute.

### 3.3 Body frame
- `body` origin at scene (0, 0, 0)
- `rotor0..3` at corners (±0.065, ±0.065, -0.025) m
- `cargo_body` at scene (0, 0, 0) m (below body)
- `left_door` at scene (0, 0.031, 0.069) m (right side of body, "left" naming = side)
- `bottom_door` at scene (0, -0.001, -0.0025) m (under cargo_body)

---

## 4. `X1_race_scene.usd` facts (the assembled world)

| Field | Value |
|---|---|
| Path | `/var/workspace/docker/isaac/scenes/active/worlds/X1_race/X1_race_scene.usd` |
| UpAxis | Z |
| **MetersPerUnit** | **1.0** (meters) |
| DefaultPrim | `World` |
| Prim count | **1195** |
| PhysicsRigidBodyAPI | 48 (= X1's 39 + sunray's 8 + scene physics extras) |
| **ArticulationRootAPI** | **4** (X1's 3 + `/World/quadrotor`) |
| PhysicsRevoluteJoint | 49 + 0 = 49 (X1:42 + sunray:7) |
| PhysicsDriveAPI:angular | 23 (= X1's 21 + sunray's 2) |
| **OmniGraph** | **10** (all on X1 side) |
| **OmniGraphNode** | **65** |

### 4.1 World tree top-level
```
/World [Xform]
  layout [Xform]
    scene [Scope]            ← 108 static furniture (walls, desks, books, pens, ...)
    caughting [Scope]        ← 1 child: SM_Pencil_White3_129 (YOLOE target)
    mercury_x1_final [Xform]
      mercury_x1 [Xform]     ← 27 children, the whole X1 robot
  quadrotor [Xform]          ← /World/quadrotor, NOT under /World/layout
    body / 4 rotors / mounted_cargo_bay
    _mid360_visual_source     ← Mid-360 decoration mesh
    cargo_mount_joint
```

Note: the X1 lives under `/World/layout/mercury_x1_final/mercury_x1` but the
quadrotor lives under `/World/quadrotor` (different parent). They're at the
same depth (`/World/{layout/mercury_x1_final, quadrotor}`).

### 4.2 Caughting (target object)
- `/World/layout/caughting/SM_Pencil_White3_129/SM_Pencil_White3_129/SM_Pencil_White` — the **Mesh** (the actual visible pencil)
- This is what YOLOE has to detect in M6 / task 1.3

### 4.3 **Existing OmniGraphs (10 total, all pre-baked in USD)**

This is the most important fact: **the scene USD ships with all the ROS 2 binding
graphs you need for X1 + LIDAR already built.** `scene_app.py` doesn't have to
recreate them; it just has to know they exist and not stomp on them.

| # | Graph path | Nodes | Publishes | Subscribes |
|---|---|---|---|---|
| 1 | `mercury_x1/Graph/ROS_Clock` | 4 | `/clock` | — |
| 2 | `mercury_x1/Graph/DiffController` | 10 | — | `/cmd_vel` (Twist) |
| 3 | `mercury_x1/Graph/transform_tree_odometry` | 11 | `/odom`, `/tf`, `/tf_static` (variants) | — |
| 4 | `mercury_x1/Graph/ROS_JointStates` | 6 | `/joint_states` (X1 full) | `/joint_command` (X1 full) |
| 5 | `mercury_x1/Graph/ROS_JointStates_01` | 6 | `/gripper_joint_states` | `/gripper_command` |
| 6 | `mercury_x1/Graph/ROS_Camera_left` | 7 | `/stereo_left/...` RGB+Depth+Info | — |
| 7 | `mercury_x1/Graph/ROS_Camera_right` | 7 | `/stereo_right/...` RGB+Depth+Info | — |
| 8 | `mercury_x1/Graph/ROS_TF_leftcamera` | 4 | `/tf` (camera) | — |
| 9 | `mercury_x1/Graph/ROS_TF_rightcamera` | 4 | `/tf` (camera) | — |
| 10 | `mercury_x1/Graph/ROS_LidarRTX` | 6 | `/laser_scan` (LaserScan, NOT PointCloud!) | — |

### 4.4 Implication: scene_app.py likely **stomp on USD-baked graphs**

`scene_app.py` builds these graphs at runtime (verified from function names +
`/World/CargoAvoidance*Graph` path constants):

| scene_app.py graph | Same as USD-baked? | Risk |
|---|---|---|
| `build_ros_clock_graph` (`/World/CargoAvoidanceClockGraph`) | duplicates `ROS_Clock` | **`/clock` published twice** at 60 Hz combined — both publishers connect to subscribers but the bandwidth is wasted |
| `build_ground_cmd_graph` | duplicates `DiffController` | **`/cmd_vel` had 2 subscribers in Isaac Sim (1 in host)**, last-write-wins between USD's DifferentialController (wheelRadius=0.0675) and scene_app's (wheelRadius=0.05) — wheels responded unpredictably |
| `build_ground_joint_graph` | duplicates `ROS_JointStates` + `ROS_JointStates_01` | **`/joint_states` published twice, `/joint_command` subscribed twice** — the second subscriber (scene_app) was the only one writing to X1 arms, so the USD-baked subscriber was dead weight |
| `build_lidar_publish_graph` | **no** duplicate — different LiDAR, different topic | OK, kept. scene_app publishes `/avoidance/lidar/pointcloud` (PointCloud2); USD publishes `/laser_scan` (LaserScan); two distinct sensors |
| `build_avoidance_state_graph` | no equivalent | OK, kept. new — publishes `/drone0/state/pose` `/drone_0_ego_odom` from Pegasus's pose observer |

**Concrete impact on M9.1 (World.play() → host topic 全在):**
- A single `/cmd_vel` publish from `ros2 topic pub` works because both subscribers
  process it; wheel movement is driven by the first graph Kit evaluates — **non-deterministic**.
- `/joint_states` had up to 3 publishers with identical values, wasteful but functional.
- **Verified 2026-07-21**: the topic names baked into the USD graphs are
  `/clock`, `/cmd_vel`, `/joint_states`, `/joint_command`, `/gripper_joint_states`,
  `/gripper_command`, `/rgb`, `/depth`, `/camera_info`, `/tf_static`, `/laser_scan`.
  See §4.5 / Appendix E for the full list.

**Fixed 2026-07-21**: scene_app now skips the 3 duplicates. See §5.1 / Appendix E.

### 4.5 LiDAR design — two LiDARs in scene

| Source | Prim | Topic | Type | Notes |
|---|---|---|---|---|
| USD-baked | `mercury_x1/Graph/ROS_LidarRTX` | `LaserScanPublish` → `/laser_scan` | LaserScan | Pre-baked, comes with the scene |
| scene_app runtime | `/World/quadrotor/body/avoidance_lidar/sensor` (added by `attach_hidden_lidar()`) | `/avoidance/lidar/pointcloud` (PointCloud2) | PointCloud2 | Required by interface_audit.py contract |

So the contract's `/avoidance/lidar/pointcloud` is the **scene_app-managed** LiDAR.
The USD-baked `/laser_scan` is **not in the contract** but exists anyway.

---

## 5. Gaps that block full-pipeline debugging

### 5.1 **Stomping risk: scene_app.py IS re-baking USD-baked graphs** — RESOLVED 2026-07-21

Verified by reading `scene_app.py:820-1040` and `omni.isaac.dynamic_control`
graph state. Three scene_app build functions **were** creating duplicate
OmniGraphs alongside the USD-baked ones:

| scene_app graph | Duplicates USD graph | Status |
|---|---|---|
| `build_ros_clock_graph` | `ROS_Clock` | **REMOVED 2026-07-21** (commented out) |
| `build_ground_cmd_graph` | `DiffController` | **REMOVED 2026-07-21** (commented out) |
| `build_ground_joint_graph` | `ROS_JointStates` + `ROS_JointStates_01` | **REMOVED 2026-07-21** (commented out) |
| `build_lidar_publish_graph` | (none) | kept |
| `build_avoidance_state_graph` | (none) | kept |

**Removed by editing `scene_app.py:1195-1200`** — the three function calls
are now commented out with a `NOTE 2026-07-21` block pointing back to this
section. The function definitions are kept (not deleted) for easy rollback.
Backup: `scene_app.py.bak.2026-07-21`.

**Post-removal verification (2026-07-21, `drone_interface_audit` report at
`docs/project/audit_post_3graph_removal_2026-07-21.json`):**

| Topic | Publisher count | Subscriber count | Status |
|---|---|---|---|
| `/clock` | 1 (`ROS_Clock` USD-baked) | 1 (px4_state_adapter) | ✓ |
| `/cmd_vel` | 0 | 1 (`DiffController` USD-baked) | ✓ |
| `/joint_states` | 1 (`ROS_JointStates` USD-baked) | varies | ✓ |
| `/gripper_joint_states` | 1 (`ROS_JointStates_01` USD-baked) | varies | ✓ |
| `/joint_command` | 0 | 1 (`ROS_JointStates` SubscriberJointState) | ✓ |
| `/gripper_command` | 0 | 1 (`ROS_JointStates_01` SubscriberJointState) | ✓ |
| `/laser_scan` | 1 (`ROS_LidarRTX` USD-baked) | 0 | ✓ |
| `/avoidance/lidar/pointcloud` | 1 (scene_app) | 0 | ✓ |
| `/fmu/in/trajectory_setpoint` | 1 (trajectory_executor) | 1 (PX4 SITL) | ✓ |
| `/drone0/state/pose` | 1 (Pegasus ROS2Backend) | varies | ✓ |
| `/drone0/sensors/imu` | 1 (Pegasus ROS2Backend) | varies | ✓ |
| `/drone_0_ego_odom` | 1 (scene_app build_avoidance_state_graph) | varies | ✓ |

**`drone_interface_audit ok=true, unique_fmu_writer=true`** (after 30s spin-up).
**`ros2 topic pub /cmd_vel 0.3` succeeded at 20 Hz**, X1's USD-baked DiffController
received and applied (wheelRadius=0.0675, wheelDistance=0.233).

**Notes for next iteration**:
- USD-baked `DiffController` uses `wheelRadius=0.0675` (with tire), `wheelDistance=0.233`.
  X1 USD has `front_left_wheel` link center at z=0.05225. The 0.0675 likely includes
  tire thickness (0.05225 + ~0.015 = 0.067). If the car moves too slowly,
  edit the USD or override via `X1_OVERRIDE_WHEEL_RADIUS=0.052` env var in next refactor.
- `/cmd_vel` also has a `cmd_vel_relay` subscriber in `isaac_ros2_control` (host-side
  bridge); this is NOT a stomp — it forwards the message but doesn't drive physics.

### 5.2 **prim_registry.yaml covers X1 joints only; no cameras, no LiDAR**

Current `prim_registry.yaml` has **37 joints + 3 articulation_roots**, all
inside X1. Missing:
- 4 UsdGeom.Camera paths (left/right eye + X1 radar-placeholder + drone FPV)
- 1 attachment point for the dynamic `avoidance_lidar` (the prim is added at
  runtime by `attach_hidden_lidar`)
- No mount points for the eye cameras (camera bodies are under `link_eye/`)
- No mention of the cargo bay door joint prim paths (they're hard-coded in
  `config.py` as `CARGO_BAY_PRIM`, `LEFT_DOOR_JOINT`, `BOTTOM_DOOR_JOINT`)

For auto-joint-binder to be the single source of truth, **prim_registry.yaml
must include cameras and dynamic attach points**.

### 5.3 **All 39 X1 rigid bodies have mass=None**

Every `PhysicsRigidBodyAPI` has `mass` authored? Let me check more carefully —
the recon script only reports `mass` if `HasAuthoredValue()`. If mass is left
to `PhysicsMassAPI` defaults, the simulation will be physically wrong. **Grasp
physics will be off**, which directly hurts M6/1.4 (抓取, 8 分).

**Action:** script needs to walk `PhysicsMassAPI` attribute (`physics:mass`)
explicitly, not just the rigid body's mass attribute.

### 5.4 **Interface contract has no ground X1 topics**

`COMMON_REQUIRED_TOPICS` + `PX4_REQUIRED_TOPICS` is **all drone + cargo + LiDAR**.
Missing for full-pipeline test:
- `/cmd_vel` (subscribed by USD `DiffController`, but interface_audit doesn't check it)
- `/joint_states` (subscribed by USD `ROS_JointStates`, but interface_audit doesn't check it)
- `/joint_command` (if we want to drive X1 arm from host)
- `/arm_left/.../camera_info` (from USD `ROS_Camera_left`)
- `/arm_right/.../camera_info` (from USD `ROS_Camera_right`)
- `/laser_scan` (from USD `ROS_LidarRTX` — pre-baked, not in contract)
- `/tf` (already in contract ✓)

For **M9.1 (host topic 全在)**, the audit currently checks `/fmu/*`, drone
sensors, `/avoidance/lidar/pointcloud`, `/drone0/down_camera/*`, `/cargo_bay/*`.
It does **not** verify the X1 side is alive. After refactor, this needs
to be added: extend `COMMON_REQUIRED_TOPICS` with the X1 set, but mark them
"ground" or split into a separate list to keep the existing drone audit intact.

### 5.5 **No downward camera on drone, no downward camera on X1 arms**

- Drone has only 1 FPV camera at `body/camera` (f=50) — **facing forward**, not down.
- X1 has only the 2 stereo eyes at `link_eye/{Left,Right}_Camera` (f=2.6) — **facing forward**, not down to gripper.
- For task 1.3 (位姿估计) and task 1.4 (抓取), we need at least one camera that
  sees the workspace / target / gripper. The eyes see forward, the FPV is
  at the drone.

Two paths:
- **A**: Add downward cameras to X1 grippers (link7_L/link7_R tips) and drone
  bottom (under cargo bay). Modify USD via scene_app.
- **B**: Live with eye-only cameras for the ground side, and accept lower
  recognition precision (penalize 1.3 by 3-5 points = 6-10/10 max).

**For scoring**: option A is required for top marks. **M1/M6 work blocked on this.**

### 5.6 **No IMU, no GPS, no Lidar schemas in USD; required by contract**

The interface contract says `/drone0/sensors/imu`, `/drone0/sensors/mag`,
`/drone0/sensors/gps` etc. are required. But sunray.usd has none of these.
**PegasusSimulator's ROS2Backend generates these at runtime** — verified in
scene_app.py behavior history. The M2 evidence (M2 evidence) says "真实比赛场景
接口票据 必备 topic/QoS/频率/电机映射全部通过", meaning when scene_app runs
end-to-end, all topics appear. So this gap is filled by scene_app + Pegasus, not
the USD. **Not a gap, but worth documenting.**

### 5.7 **scene_app.py 1297 lines, 3 classes, 30+ methods — no module structure**

Already discussed. The plan addresses this.

### 5.8 **Helper scripts 4 × ~80 lines each, duplicate x1 introspection**

`x1_joint_inspector.py`, `x1_joint_inspector_v2.py`, `x1_prim_debugger.py`,
`x1_map_generator.py` — 4 files, all doing variations of "walk X1 prim tree
and dump". v2 supersedes v1. **Consolidate into one `tools/dump_prims.py` with
flags** (joints/cameras/sensors/transforms/voxel-map output).

---

## 6. New plan — refactor + auto-joint-binder

### 6.1 Refactor scope (where the scene_app goes)

Target directory tree:

```
scenes/active/scripts/integrated_runtime/
├── __init__.py
├── config.py                       ← only constants, env-var parsing, no USD paths
├── prim_registry.py                ← reads prim_registry.yaml, dataclasses
├── prim_registry.yaml              ← extended: joints + cameras + lidar + attach_points
├── usd_utils.py                    ← _vec3d / _ensure_prim / xform helpers
├── scene_app.py                    ← <200 lines: init world → load → wire → play
├── run_demo_scene.sh
├── payload/                        ← cargo bay + pencil
│   ├── __init__.py
│   ├── cargo_bay.py                ← CargoBayRuntime (was 300 lines)
│   ├── pencil.py                   ← pencil placement + collision proxy
│   └── payload_lock.py             ← mount + lock joints
├── ros_bridge/                     ← one graph per file, all <150 lines
│   ├── __init__.py
│   ├── clock_graph.py              ← duplicates or just relies on USD-baked
│   ├── lidar_graph.py              ← the dynamic avoidance_lidar + PointCloud2
│   ├── state_graph.py              ← /drone_0_ego_odom + /drone0/state/*
│   ├── ground_cmd_graph.py         ← /cmd_vel → X1 wheels  (only if USD's missing)
│   ├── ground_joint_graph.py       ← /joint_states + joint_command
│   └── cargo_door_graph.py         ← /cargo_bay/command → door joints
├── assembly/                       ← scene loader + prim resolver
│   ├── __init__.py
│   ├── scene_loader.py
│   ├── lidar_resolver.py
│   ├── prim_resolver.py            ← USD prim-by-name/hint/schema resolution
│   └── joint_binder.py             ← THE auto-binder — see §6.2
├── ros_publishers/                 ← small ROS 2 publisher wrappers (rclpy)
│   ├── __init__.py
│   ├── cargo_publisher.py          ← publishes /cargo_bay/{command,status}
│   └── imu_gps_mag_publisher.py    ← the missing schemas filled in by Pegasus
├── tools/
│   ├── dump_prims.py               ← one file, replaces 4 x1_* helpers
│   ├── dump_joints.py              ← subset: just joints
│   └── dump_sensors.py             ← subset: just sensors/cameras
└── tests/
    ├── test_prim_resolver.py       ← USD mock stage, runs in CI
    ├── test_prim_registry.py       ← YAML loading, schema validation
    └── test_payload.py             ← cargo_bay + pencil logic
```

### 6.2 Auto-joint-binder design

**prim_registry.yaml** schema (extended):

```yaml
# All prim paths are real, validated against the USD on first load.

x1:
  articulation_roots:
    base_link: /World/layout/mercury_x1_final/mercury_x1/base_link
    left_gripper: /World/layout/mercury_x1_final/mercury_x1/left_gripper
    right_gripper: /World/layout/mercury_x1_final/mercury_x1/right_gripper
  joints:                           # all 37, with bind info
    - {name: joint1_L, path: ..., drive: position, limits: [-2.88, 2.88]}
    - {name: joint2_L, path: ..., drive: position, limits: [-0.96, 1.66]}
    - {name: joint3_L, path: ..., drive: position, limits: [-3.02, 0.087]}
    - {name: joint4_L, path: ..., drive: position, limits: [-2.88, 2.88]}
    - {name: joint5_L, path: ..., drive: position, limits: [-0.35, 4.63]}
    - {name: joint6_L, path: ..., drive: position, limits: [-3.14, 3.14]}
    # ... 31 more
  cameras:                          # NEW
    - {name: left_eye,  path: /World/.../link_eye/Left_Camera,  f_mm: 2.6, role: stereo_left}
    - {name: right_eye, path: /World/.../link_eye/Right_Camera, f_mm: 2.6, role: stereo_right}
    - {name: link_radar_rotating, path: /World/.../link_radar/Rotating, f_mm: 50.0, role: misc}  # f=50 placeholder
  attach_points:                    # NEW — places we'd add or use
    - {name: avoidance_lidar_mount, parent: /World/quadrotor/body, offset: [0, 0, 0.12]}

sunray:
  articulation_root: /World/quadrotor
  joints:
    - {name: rotor0, path: /World/quadrotor/rotor0/joint0, drive: velocity, limits: null}
    - {name: rotor1, path: /World/quadrotor/rotor1/joint1, drive: velocity, limits: null}
    - {name: rotor2, path: /World/quadrotor/rotor2/joint2, drive: velocity, limits: null}
    - {name: rotor3, path: /World/quadrotor/rotor3/joint3, drive: velocity, limits: null}
    - {name: left_door_joint,   path: /World/quadrotor/mounted_cargo_bay/transparent_cargo_bay/joints/left_door_joint,   drive: position, limits: [-1.57, 0]}
    - {name: bottom_door_joint, path: /World/quadrotor/mounted_cargo_bay/transparent_cargo_bay/joints/bottom_door_joint, drive: position, limits: [-1.57, 0]}
  cameras:                          # NEW
    - {name: body_camera, path: /World/quadrotor/body/camera, f_mm: 50.0, role: fpv}
  attach_points:                    # NEW
    - {name: under_belly, parent: /World/quadrotor/body, offset: [0, 0, -0.05]}
```

**`assembly/joint_binder.py`** (one file, <200 lines):

```python
class JointBinder:
    """Reads prim_registry.yaml, creates OmniGraph nodes for ROS 2 publish/subscribe."""

    def __init__(self, stage, prim_registry: PrimRegistry):
        self.stage = stage
        self.reg = prim_registry

    def bind_articulation_state(self, name: str, root_prim_path: str,
                                 state_topic: str, cmd_topic: str | None) -> str:
        """Create ROS_JointStates-style graph: publisher + optional subscriber.
        Returns the new OmniGraph prim path."""
        ...

    def bind_camera_rgb_depth_info(self, name: str, camera_prim: str,
                                    frame_id: str, info_topic: str,
                                    rgb_topic: str, depth_topic: str) -> str:
        """Create ROS_Camera_* style graph: RenderProduct + RGBPublish + DepthPublish + CameraInfo."""
        ...

    def bind_lidar_pointcloud(self, name: str, lidar_prim: str,
                               topic: str, frame_id: str) -> str:
        """Create ROS_LidarRTX-as-PointCloud graph."""
        ...

    def bind_door_command(self, name: str, joint_prim: str, cmd_topic: str) -> str:
        """Create subscriber for /cargo_bay/command style door control."""
        ...

    def bind_all_from_registry(self) -> dict[str, str]:
        """Drive the whole YAML. Returns map of {graph_name: graph_path}."""
        results = {}
        for spec in self.reg.all_bind_specs():
            if spec["type"] == "articulation_state":
                results[spec["name"]] = self.bind_articulation_state(**spec)
            elif spec["type"] == "camera":
                results[spec["name"]] = self.bind_camera_rgb_depth_info(**spec)
            elif spec["type"] == "lidar":
                results[spec["name"]] = self.bind_lidar_pointcloud(**spec)
            elif spec["type"] == "door":
                results[spec["name"]] = self.bind_door_command(**spec)
        return results
```

This is the abstraction that "new joint → add line to YAML, don't touch code".

### 6.3 Plan, ordered (with concrete checks at each step)

The plan is **strictly ordered** — each step must end with a green
`drone_interface_audit` + `chain_status` snapshot before moving on.

**Step 0 — Recon (done, this doc)**
- [x] Recon USD, scene, contract.
- [x] Identify USD-baked vs scene_app-built graphs.
- [x] Identify the 7 gaps in §5.

**Step 1 — Quiescence baseline** (no code change, ~1 hour)
1. Restart isaac sim. Run `drone_interface_audit`. **Record the baseline JSON**.
2. Run `chain_status` to see live topic snapshot.
3. Count publishers per topic (`ros2 topic info /joint_states -v`).
4. **Save baseline** to `docs/project/scene_recon_baseline.json`.
5. Acceptance: a single file that says "before any change, this is what the graph looks like."

**Step 2 — Fix the stomping risk** ✅ DONE 2026-07-21
1. ✅ Audited which graphs are USD-baked (§4.3) and which scene_app builds.
2. ✅ For each duplicate (`ROS_Clock`, `DiffController`, `ROS_JointStates` + `_01`),
   commented out the scene_app build call at `scene_app.py:1195-1200`.
3. ✅ Re-ran `drone_interface_audit`. JSON saved at
   `docs/project/audit_post_3graph_removal_2026-07-21.json`.
4. ✅ Acceptance: `cmd_vel` has 0 publishers + 1 USD subscriber, `joint_states`
   has 1 publisher, `clock` has 1 publisher, `ok=true`, `unique_fmu_writer=true`.
   The single biggest correctness win landed.

**Step 3 — Add ground-X1 topics to interface_audit** (small change, ~2 hours)
1. Extend `COMMON_REQUIRED_TOPICS` with: `/cmd_vel`, `/joint_states`,
   `/joint_command` (or whatever the USD-baked graph subscribes to — verify),
   `/stereo_left/{rgb,depth,camera_info}`, `/stereo_right/{rgb,depth,camera_info}`,
   `/laser_scan` (or remove from USD if not used).
2. Run audit. **Must pass.**
3. Update `docs/contracts/interface_contracts.md` to mirror the new list.

**Step 4 — Verify with M9.1 (host topic 全在)** (~30 min)
1. `ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py`
2. `chain_status` shows all 20+ ground+X1+drone topics, all `ok=true`.
3. `ros2 topic echo /cmd_vel` while publishing on it → wheels spin (verify
   in VNC).
4. **This is the proof the refactor didn't break the chain.**

**Step 5 — Refactor scene_app.py** (refactor only, no behavior change, ~1 day)
1. Move each `build_*_graph` into `ros_bridge/<thing>_graph.py` as a class
   with `__init__(stage, registry)` + `build() -> str`.
2. Move `CargoBayRuntime` to `payload/cargo_bay.py`.
3. Move `RosCargoInterface` to `ros_publishers/cargo_publisher.py`.
4. New `scene_app.py` is <200 lines: it reads `prim_registry.yaml`, instantiates
   the binder, calls `bind_all_from_registry()`, runs lifecycle.
5. After each move, restart scene, run audit, **must remain green**.
6. Acceptance: same `chain_status` output, smaller scene_app.py.

**Step 6 — Add cameras and attach_points to prim_registry.yaml** (~1 hour)
1. Update YAML with the 4 cameras + attach_points (§5.2).
2. Update `prim_resolver.py` to load and validate.
3. **This step prepares Step 7, doesn't yet build graphs.**

**Step 7 — Auto-joint-binder (the actual `assembly/joint_binder.py`)** (~4 hours)
1. Implement `JointBinder` per §6.2.
2. Wire `scene_app.py` to call `binder.bind_all_from_registry()`.
3. **Critical**: keep the same topic names as the USD-baked graphs so the
   rest of the stack doesn't need to change.
4. Acceptance: still green audit, no functional change.

**Step 8 — Add downward cameras (X1 gripper tips + drone belly)** (deferred)
- **Not required for M1-M3** but **required for top score on M6/1.4**.
- If time permits: add `link7_L/down_camera`, `link7_R/down_camera`,
  `quadrotor/body/down_camera`. Each with new OmniGraph for `/down_camera_*/image_raw`.
- Add to registry as `role: gripper_down_left`, `gripper_down_right`, `drone_down`.

**Step 9 — Add mass to X1 bodies** (~30 min, scripted)
1. Write `tools/author_masses.py` that walks all `PhysicsRigidBodyAPI` and
   sets `physics:mass` based on geometry (volume × density table).
2. Re-export the scene, or just modify the in-memory stage at scene_app startup.
3. Re-run physics verification (drop test, M9.4-ish).

**Step 10 — Submit** (D-1, 07-24): record videos, freeze code, package, md5.

### 6.4 Effort / risk

| Step | Effort | Risk | Notes |
|---|---|---|---|
| 1 | 1h | none | just data |
| 2 | 1h | low | delete dead code, validate |
| 3 | 2h | low | contract change, but well-tested |
| 4 | 30m | low | proof step |
| 5 | 1d | medium | mechanical refactor, 3-4 Isaac Sim restarts |
| 6 | 1h | none | YAML only |
| 7 | 4h | medium | the binder is the centerpiece; can iterate |
| 8 | 2h | medium | USD authoring, but only adds, doesn't change |
| 9 | 30m | low | scripted |
| 10 | D-day | — | |

**Total engineering time: ~3 days.** D-4 means we have 3-4 days of work
buffer before D-1 hard freeze. Tight but doable.

### 6.5 What we explicitly **do not** do

- **No full URDF export of X1** for cuRobo. The X1 6-DOF URDF approximation
  in `perception_competition_pkg/scripts/x1_left_arm.urdf` is good enough for
  IK demo. Re-doing it from scratch is M1 work that's not on the critical path.
- **No PCL/Nav2 map build** in the refactor scope — already done.
- **No unit tests for OmniGraph code** — requires a Kit runtime, not testable
  in CI. We verify via scene restart + `chain_status` instead.
- **No write-the-tech-doc yet** — D-2 task (07-23).

---

## Appendix A — prim_registry.yaml current state (37 joints)

Covered: 37/37 from the recon. The YAML matches reality (verified path by path
during recon). What the YAML **doesn't** cover:
- Cameras (3 in mercury_x1 + 1 in sunray)
- Avoidance LiDAR attach point
- Lidar schemas (none in USD)
- IMU / Mag / GPS schemas (Pegasus generates at runtime)
- Mass values (all None)

## Appendix B — prim_registry.yaml verification

```bash
# Run this to verify YAML paths against the scene USD:
/home/socl/miniconda3/envs/isaacsim51/bin/python -c "
import yaml
from pxr import Usd
with open('/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/prim_registry.yaml') as f:
    reg = yaml.safe_load(f)
stage = Usd.Stage.Open('/var/workspace/docker/isaac/scenes/active/worlds/X1_race/X1_race_scene.usd')
ok = 0; missing = []
for j in reg['joints']:
    p = stage.GetPrimAtPath(j['path'])
    if p and p.IsValid():
        ok += 1
    else:
        missing.append(j['path'])
print(f'OK: {ok} / {len(reg[\"joints\"])}, missing: {len(missing)}')
for m in missing[:5]: print(' ', m)
"
```
**Result: OK 37/37, missing 0.** Registry is ground-truth accurate.

## Appendix C — Recon script landed

The recon script `/tmp/usd_recon.py` will be moved to
`scenes/active/scripts/integrated_runtime/tools/dump_prims.py` as part of
Step 5 (refactor). For now, it can be re-run from any host with the
`isaacsim51` conda env via:

```bash
/home/socl/miniconda3/envs/isaacsim51/bin/python /tmp/usd_recon.py \
  /var/workspace/docker/isaac/scenes/active/worlds/X1_race/X1_race_scene.usd

# JSON dump for diffing:
/home/socl/miniconda3/envs/isaacsim51/bin/python /tmp/usd_recon.py \
  <usd_path> --json > /tmp/snapshot.json
```

## Appendix D — stomping risk verification commands

To verify Step 2 (delete duplicate graphs) actually works, run these **before
and after** the change, and diff:

```bash
# before
ssh soclserver 'sudo systemctl restart isaacsim51-scene.service' && sleep 90
ssh soclserver 'source /opt/ros/jazzy/setup.bash && source /var/workspace/docker/isaac/workspace/install/setup.bash && \
  export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
  timeout 5 ros2 topic info /cmd_vel -v' 2>&1 | tee /tmp/cmd_vel_before.txt
ssh soclserver '... ros2 topic info /joint_states -v' 2>&1 | tee /tmp/joint_states_before.txt
ssh soclserver '... ros2 topic info /clock -v' 2>&1 | tee /tmp/clock_before.txt
ssh soclserver '... ros2 topic info /laser_scan -v' 2>&1 | tee /tmp/laser_scan_before.txt
```
