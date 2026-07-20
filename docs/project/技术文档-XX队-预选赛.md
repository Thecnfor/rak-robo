# 技术文档 — 双臂 + 无人机协同货舱交付
## 第二十五届全国大学生机器人大赛 ROBOTAC AIROBOTIC 赛项 预选赛

> 占位：`<队名>` → XX队；提交前请用 `sed` 全局替换。

---

## 1. 仿真场景配置

### 1.1 软硬件栈

| 组件 | 版本 | 部署位置 |
|---|---|---|
| Isaac Sim | 5.1.0 | 主机 `conda env isaacsim51` |
| PegasusSimulator | 5.1 (Multirotor backend) | 跟随 Isaac Sim 启动 |
| cuRobo | 0.8.0 | Isaac Sim Python env |
| ROS 2 Jazzy | desktop | `/opt/ros/jazzy` |
| RMW | `rmw_fastrtps_cpp` (Fast DDS) | Domain ID **45** |
| PX4 SITL | v1.16.2 | Docker 容器 `px4-sitl` (`/var/workspace/docker/isaac/docker/px4/`) |
| `px4_msgs` | v2.0.1 (PX4 main 分支) | 主机 colcon workspace |
| NVIDIA GPU | Tesla T4 (16 GB VRAM) | 主机 |

### 1.2 启动流程

`docs/setup/isaacsim_scene_daemon.md` 详述，本节摘要：

1. **基础 daemon**（Xvfb + x11vnc）：`xvfb-isaac.service`、`x11vnc-isaac.service`。
2. **场景 daemon**：`isaacsim51-scene.service` 启动 `isaacsim ... --exec scene_app.py --world X1`，挂载 cargo scene、Pegasus `PX4MavlinkBackend`（TCP 4560）、ROS 2 bridge。
3. **PX4 SITL Docker**：`cd /var/workspace/docker/isaac/docker/px4 && docker compose up -d`；通过 `PX4_SIM_MODEL=...` 切换 Pegasus 模式 (`gazebo-classic_iris`) 或主机模式 (`sihsim_quadx`)。
4. **Host bringup**：`ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py`，一键拉起 4 飞行节点 + 2 视觉节点 + 1 编排 + 3 dual-arm + 1 审计 + Foxglove。
5. **Foxglove Studio**：`ws://localhost:8765`（systemd --user 常驻）。

### 1.3 VNC 与 Play

- VNC `:5900` 密码 `robo2026`，观察仿真。
- 场景 Play 由 daemon 中 `scene_app.py::timeline.play()` 自动调用；比赛主链 VNC 仅用于观察/手动调试。

### 1.4 RTX 调优

`isaacsim51-scene.service.d/99-tuning.conf` 注入：

```
--/app/window/width=1920 --/app/window/height=1080
--/app/window/fullscreen=true --/app/window/decorations=false
--/rtx/quality/dlssMode=0 --/rtx/quality/preset=MaxQuality
--/rtx/quality/aaMode=0
--/renderer/skipWhileMinimized=false
--/renderer/multiGpu/enabled=0
```

实测 Tesla T4：默认 320×240 + DLSS → 调优后 1920×1080 native，VRAM 从 97 MiB → 4 GB，GPU util 0% → 67%+。

---

## 2. 视觉对准算法

### 2.1 地面抓取（grasp pipeline）

参见 `docs/project/diagrams/grasp_pipeline.png` 数据流图。

**节点**（`src/perception_competition_pkg/`）：

1. **YOLOE 检测**：`yoloe_detector_node` 加载 `weights/yoloe-26l-seg.pt`（git-ignored），订阅 `/arm_camera/rgb` → 发布 `/demo_grasp/{object_mask, bbox, label, confidence, debug_image}`。当前实现是 stub 模式（合成 200×200 框），真实部署时把 `_publish_synthetic` 替换为 `ultralytics.YOLO(...).predict(...)` 即可。
2. **深度位姿**：`depth_pose_estimator_node` 订阅 `/arm_camera/depth` + `/arm_camera/camera_info` + bbox，用 median-depth 投影到 camera-frame XYZ，PCA 计算 long_axis → 发布 `/demo_grasp/{object_point_camera, long_axis_camera}`。
3. **TF 转换**（`grasp_demo_pkg::tf_transform_demo_node`）：camera-frame → `base_link_arm`，输出 `/demo_grasp/{object_point_base, normal, long_axis_base}`。
4. **位姿规划**（`grasp_demo_pkg::plan_to_pose_node`）：用 normal+long_axis 构造候选 `PoseStamped`（pre-grasp, grasp, lift, pre-place, place），发 `/hand_command`。
5. **夹爪**（`grasp_demo_pkg::gripper_demo_node` + `dual_arm_pkg::dual_gripper_server_node`）：服务 `/demo_gripper_command` action，左/右夹爪独立或同步。

**Action 接口**（`grasp_demo_interfaces/action/`）：

- `DetectObject` — `(target_classes, confidence_threshold, timeout) → (object_mask, bbox)`
- `PlanToPose` — `(target_point, normal, long_axis) → (success)`
- `GripperCommand` — `(command, position, speed) → (success, final_position)`
- `DroneFlightCommand` — `(TAKEOFF|GOTO|RETURN|HOVER|LAND|ABORT) → (final_pose)`
- `CargoDoorCommand` — `(left|bottom × open|close) → (acknowledgement)`

### 2.2 无人机下视对准（drone vision）

节点：`drone_target_detector_node`（`perception_competition_pkg/`）。

- 订阅 `/drone0/down_camera/color/image_raw`，HSV 红环 mask + minEnclosingCircle → 归一化 `(nx, ny, area_fraction, radius_px)`。
- 发布 `/drone/drop_target_offset`（`Float32MultiArray`），flight_supervisor 收到稳定 ≥ 0.8 s 后转 `VISUAL_ALIGN` → `DROP_HOLD`。
- 调参：`min_area=300`, `min_circularity=0.70`, `morphology_kernel=5`, `center_threshold=0.04`（`perception_competition_pkg/config/drone_target.yaml`）。

### 2.3 双目识别（加分项，A 平台专属）

YOLOE 在左/右臂相机分别推理，比较 bbox 视差得到 3D 距离。`grasp_demo_pkg` 已留有 `/arm_camera/rgb_{left,right}` topic 入口，待 `weights/yoloe-26l-seg.pt` 部署后激活。

---

## 3. 状态机图

### 3.1 任务调度（`mission_gate.png`）

```
IDLE → READY → DISPATCH → FLIGHT → COMPLETE
                      ↘ GROUND
            FLIGHT ↘ ABORT
```

- `MissionGate.observe(ground_state)` 在 `ground_state ∈ {COMPLETE, SUCCESS, GROUND_DONE}` 时**只触发一次** `mission_request=true`。
- 比赛重置靠 supervisor 发 `/arena/ground/state=IDLE`，不重入 IDLE。

### 3.2 飞行控制（`flight_supervisor.png`）

```
IDLE → PREFLIGHT → ARMING → TAKEOFF → EGO_TRANSIT → TARGET_SEARCH →
VISUAL_ALIGN → DROP_HOLD → RETURN → LAND → COMPLETE
                ↘ HOLD → EMERGENCY_LAND
```

11 个主相位 + HOLD 应急分支。

- **安全门**：stale data > 0.3 s → HOLD；> 1.0 s 或 px4_failsafe → LAND。
- **侧舱门**：PREFLIGHT 必须 `side_door_closed`，supervisor 周期性发 `left_close` 指令。
- **底舱门**：DROP_HOLD 时 `bottom_open`，等待 `/cargo_bay/status` 含 `payload_released`。

### 3.3 抓取流水线（`grasp_pipeline.png`）

```
arm_camera → yoloe_detector → depth_pose_estimator → tf_transform_demo
   → plan_to_pose → gripper_demo → pick_place_state_machine
                                          ↓
                                  /arena/ground/state=COMPLETE
```

### 3.4 无人机链路（`drone_chain.png`）

```
Pegasus → TCP 4560 → PX4 SITL → uXRCE 8888 → host FastDDS dom 45
                                                  ↓
                       px4_state_adapter → ego_local_planner
                                          ↘            ↘
                                       executor @ 20 Hz
                                                  ↓
                                              /fmu/in/* → PX4
```

**Sole-writer 规则**：`/fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}` 三个 topic 只有 `trajectory_executor` 一个发布者。`interface_audit.py` 在 runtime 持续校验。

---

## 4. 难点与解决方案

### 4.1 难点 1：Pegasus `is_homogeneous` race 阻断 sensor 输出

**现象**：Pegasus `Multirotor(...)` 包装 sunray.usd 自带 articulation，初始化时 PhysX 拿不到 homogeneous view，dynamic_control 持续抛 `Invalid or expired body handle`，sensor 不出数据，PX4 拿不到状态。

**解决路径**：

1. **比赛主链（Pegasus 模式）**：`scene_app.py` 已经在 `setup_async` 里 `try/except world.reset_async()` 继续后续 mount，但 multirotor physics 残缺。
2. **主机端验证（SIH 模式）**：`PX4_SIM_MODEL=sihsim_quadx docker compose up -d` 启动 PX4 自带的 SIH 物理，验证 host chain 的纯 ROS 端逻辑（`mission_request` → `ARM_OFFBOARD` → takeoff → LAND）通过 90% offboard 主机逻辑正确，无需 Pegasus 物理。
3. **双模式工作流**：`docs/setup/isaacsim_scene_daemon.md` §"双模式" 详述。SIH 用于 host chain 烟测，Pegasus 用于最终视频/验收。

### 4.2 难点 2：PX4 `vehicle_status` 命名版本错位

**现象**：`px4_msgs` 是 PX4 main 分支（`VehicleStatus.msg` `MESSAGE_VERSION=1`），但文档/契约用 unversioned 名。`px4_state_adapter` 订阅 `/fmu/out/vehicle_status` 永远收不到数据。

**解决**：直接订阅 `/fmu/out/vehicle_status_v1`。`interface_audit.py::resolve_actual_topic` 自动把 unversioned 契约名解析到带 `_v1` 的实际 topic，所以契约代码仍写 `vehicle_status` 但实际看到 `vehicle_status_v1`。

### 4.3 难点 3：ROS graph 在 race 时无完整 `cargo_bay/status`

**现象**：Pegasus 多线程 race 时 cargo runtime 拿不到锁，`/cargo_bay/status` 0 Hz。`flight_supervisor::onCargoStatus` 收不到，PREFLIGHT→ARMING 卡死。

**解决**：写 `cargo_status_sim` 镜像 `left_closed/left_opened/bottom_opened/payload_released` 字符串，订阅 `/cargo_bay/command` 同步状态。比赛主链（race 修好后）可关闭此 sim。

### 4.4 难点 4：节点重启的 ghost instances

**现象**：`host_bridge_bringup.launch.py` 多次启动后，graph 里出现 N 个 `px4_state_adapter` / `ego_local_planner` 等同名节点，topic 端点混乱。

**解决**：重启前先 `pkill -f <node_name>` 清理；或用 `ros2 daemon start` + `ros2 launch` 走 lifecycle。

---

## 5. 验收证据

| 指标 | 实测 | 标准 |
|---|---|---|
| 端到端 `IDLE → PREFLIGHT → ARMING` | ✅ (D-3.2) | 必过 |
| 飞行 `TAKEOFF → LAND` | SIH 端 state:TAKEOFF (D-5) | 必过 |
| 投放落点 ≤ 0.2 m | D-3.4（用 cargo_status_sim + 视觉 PID 调参） | 90% 落点 |
| 10 次连续无碰撞 | D-3.3 概念通过（ARMING 阶段） | 必过 |
| 接口审计 `ok=true` | `drone_interface_audit` 在 SIH 模式 100% 满足 | 必过 |

---

## 6. 提交包结构

```
双臂-XX队-预选赛.zip
├── videos/                          # 10 段必交 + 5 段加分
│   ├── 预选赛赛段1任务1-仿真基础配置.mp4
│   ├── 预选赛赛段1任务2-自主导航.mp4
│   ├── 预选赛赛段1任务3-识别位姿.mp4
│   ├── 预选赛赛段1任务4-抓取.mp4
│   ├── 预选赛赛段1任务5-投放.mp4
│   ├── 预选赛赛段2任务1-仿真基础.mp4
│   ├── 预选赛赛段2任务2-关侧舱.mp4
│   ├── 预选赛赛段2任务3-起飞飞行.mp4
│   ├── 预选赛赛段2任务4-视觉对准.mp4
│   ├── 预选赛赛段2任务5-投放执行.mp4
│   ├── 预选赛赛段2任务6-精准度.mp4
│   └── (加分) 加分1-全流程.mp4 / 加分2-双目识别.mp4 / 加分3-双臂协同.mp4 / 加分4-cuRobo.mp4 / 加分5-无人机投放.mp4
├── technical_doc/
│   └── 技术文档-XX队-预选赛.pdf
└── 工程文件/
    ├── usd_scenes/
    ├── action_graphs/                # OmniGraph 截图
    ├── python_scripts/               # 4 个比赛包源码
    ├── configs/
    ├── maps/
    └── README.md
```

提交邮箱：`airobot@turingltd.com`，截止 `2026-07-25 23:59`。
