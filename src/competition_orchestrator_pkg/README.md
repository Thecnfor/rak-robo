# competition_orchestrator_pkg

> **Owner: D**
> **比赛模块: M4（空地协同调度）+ M7（货舱关节控制）+ M8（文档 + 提交工具）**
> **日期: 2026-07-17 起**

## 这是什么

**协调器包**。包括三部分：

1. **空地协同状态机**（`air_ground_coordinator`）：监听 A 的 ground state + B 的 drop command，按 phase 切换：INIT → WAIT_GROUND → GROUND_RUNNING → CLOSE_LEFT_DOOR → DRONE_TAKEOFF → FLY_TO_DROP → VISUAL_ALIGN → DROP → RETURN → COMPLETE
2. **货舱关节 action server**（`cargo_door_server`）：关侧舱 / 开底舱 / 关底舱
3. **无人机飞控 action server**（`drone_flight_server`）：把 takeoff / goto 任务交给 EGO 算法适配层和 PX4 Offboard 控制链路；PegasusSimulator 负责载具仿真，不替代 PX4 飞控
4. **辅助脚本**（在 `scenes/team/scripts/`）：
   - ffmpeg 烧水印脚本
   - 录屏脚本
   - 提交包打包脚本
   - 状态机图生成（Mermaid / Graphviz）

## 接口（D 暴露给所有 4 人的）

### 发布
- `/arena/orchestrator/state` (std_msgs/String)

### 提供 Action
- `/demo_close_left_door` (grasp_demo_interfaces/action/CargoDoorCommand)
- `/demo_open_bottom_door` (grasp_demo_interfaces/action/CargoDoorCommand)
- `/demo_drone_takeoff` (grasp_demo_interfaces/action/DroneFlightCommand)
- `/demo_drone_goto` (grasp_demo_interfaces/action/DroneFlightCommand)

### 订阅
- `/arena/ground/state` ← A
- `/drone/drop_command` ← B（drop decider 触发）
- `/drone/drop_target_offset` ← B（视觉对准参考，可选）

详见 [`/var/workspace/docker/isaac/workspace/docs/contracts/interface_contracts.md`](../docs/contracts/interface_contracts.md) § D。

## 编译

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select competition_orchestrator_pkg
source install/setup.bash
```

## 运行

```bash
# 单独跑协调器（要 A/B/C 的节点已经在跑）
ros2 run competition_orchestrator_pkg air_ground_coordinator

# 跑货舱控制（独立可测）
ros2 run competition_orchestrator_pkg cargo_door_server

# 跑水印/录屏（不进 ROS2 namespace）
/var/workspace/docker/isaac/scenes/team/scripts/add_watermark_and_record.sh "XX队-张三" output.mp4
```

## 工作分支

`orchestrator_d`（在 `/var/workspace/wt_d`）。

## 详细 TODO

见 `TODO.md`。

## 不要做

- ❌ 不要 import A/B/C 任何代码
- ❌ 不要写 YOLOE 检测（那是 B 的）
- ❌ 不要写机械臂控制（那是 A 的）
- ❌ 不要写 OmniGraph 节点（那是 C 的）
- ❌ 不要抢别人的修改（除非在团队群里 ack 过）

## 提交工具脚本位置

`/var/workspace/docker/isaac/scenes/team/scripts/`：

- `add_watermark_and_record.sh` — 录屏 + 烧水印
- `make_state_diagram.sh` — 从 README 里的 mermaid 块生成 SVG/PNG
- `package_submission.sh` — 打包总压缩包 + 命名 + md5
- `submit.sh` — 通过 curl 提交到 airobot@turingltd.com（如果邮件走 API）
