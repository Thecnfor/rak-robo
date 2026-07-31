# 赛段二截止前演示运行单

> 目标：先拿到每个任务可验证的真实画面和证据，再尝试完整飞行。不得用 SIH、`cargo_status_sim` 或手工伪造状态冒充 Pegasus 实飞。
>
> 权威安全顺序仍以 [`../runbooks/drone_navigation.md`](../runbooks/drone_navigation.md) 为准。本页只压缩现场操作步骤。

## 1. 可演示能力与证据边界

| 任务 | 优先录制的真实证据 | 当前边界 |
|---|---|---|
| 2.1 仿真基础 | X1 场景中的 Sunray、货舱、下视相机、LiDAR；`/drone0/*`、`/fmu/*`、接口审计 | 可完整展示 |
| 2.2 关侧舱 | 向真实 `scene_app.py` 发 `left_close`，画面看到门关闭，`/cargo_bay/status=left_closed` | 可完整展示；PX4 启动前必须先关闭 |
| 2.3 起飞飞行 | Pegasus + PX4 的 Offboard 预流、解锁、EGO 避障、LAND、disarm | 2026-07-31 冷启动全流程已通过；正式重复验收仍需 10 连续轮 |
| 2.4 视觉对准 | 真实下视图、红色投放标记、`/drone/drop_target_offset`、`visual_velocity` 收敛 | 最新全流程已完成空中闭环对准 |
| 2.5 投放执行 | 真实底舱门打开、payload 释放、`bottom_opened payload_released` | 最新全流程实际到达 `DROP_HOLD` 并释放 |
| 2.6 精准度 | 真实落点与圆心同屏测量；保存截图/视频和距离 | 最新静止落点误差 1.27 cm；90% 统计仍需至少 10 次投放 |

状态模拟仅允许用于“状态机/接口回退演示”，视频画面和说明必须明确写“模拟”，不能代替 2.3/2.5/2.6 的物理证据。

## 2. 现场终端准备

每个 ROS 终端先使用工作区文档规定的 Jazzy/Fast DDS 环境，然后：

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

确认环境值为：

```text
ROS_DOMAIN_ID=45
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

演示助手：

```bash
bash docs/project/stage2_demo_control.sh status
bash docs/project/stage2_demo_control.sh help
```

## 3. 冷启动顺序（收到明确“开始”后执行）

1. 停止 Host chain 与 PX4；确认无人机未解锁。
2. 确认没有第二个 Isaac/`scene_app.py` 实例，再启动唯一的 X1 场景。
3. 等 `/clock`、`/drone0/state/pose`、`/drone0/state/twist` 和 LiDAR 连续更新。
4. 在 **PX4 停止状态**关闭侧舱门和底舱门：

   ```bash
   CONFIRM_SCENE_COMMAND=YES \
     bash docs/project/stage2_demo_control.sh close-doors
   ```

5. 连续 3 秒确认出生点 `(4.55,-0.38,1.13)±0.004 m`、速度 `<0.05 m/s`、横滚/俯仰 `<0.5°`。
6. 启动 PX4 Docker，等待 `Simulator connected` 和 `Ready for takeoff!`，并确认 `/fmu/out/*` 有真实发布者。
7. 单独终端启动 Host chain 和时间戳 rosbag：

   ```bash
   bash docs/project/stage2_demo_control.sh host
   ```

8. 另一终端运行：

   ```bash
   bash docs/project/stage2_demo_control.sh audit
   ```

9. 只有接口 `ok=true`、`prearm_pose_valid=true`、`planner_map_ready=true`、舱门关闭、PX4 无 failsafe 时，才允许触发任务。
10. 真实赛段二触发只发一次地面完成事件；orchestrator 会自动派发，不能再叠加 `mission_trigger`：

    ```bash
    CONFIRM_FLIGHT=YES \
      bash docs/project/stage2_demo_control.sh mission
    ```

安全动作始终可用：

```bash
bash docs/project/stage2_demo_control.sh land
bash docs/project/stage2_demo_control.sh abort
```

## 4. 六段视频的最短录制顺序

每段开始前：

```bash
bash docs/project/stage2_demo_control.sh record-start <1..6>
```

结束：

```bash
bash docs/project/stage2_demo_control.sh record-stop
```

建议顺序：

1. **任务 1**：场景全景 → ROS topic → `chain_status`/audit。
2. **任务 2**：先展示门开状态 → 执行真实 `left_close` → 状态确认。
3. **任务 4**：下视图和红色标记 → 移动标记 → offset 收敛。
4. **任务 5**：在隔离、未飞行状态录真实底门和 payload 释放；执行前：

   ```bash
   CONFIRM_CARGO_RELEASE=YES \
     bash docs/project/stage2_demo_control.sh release-payload
   ```

5. 重载场景、重新锁定 payload、按冷启动顺序恢复。
6. **任务 3**：只在全部飞行门通过后录受控飞行；保留完整 LAND/disarm 画面。
7. **任务 6**：有真实投放才量落点；否则只保留为未完成，不伪造距离。

原始录像写入 `videos_raw/`，文件名带时间戳。处理后会得到规定的最终文件名：

```bash
bash docs/project/postprocess_videos.sh <真实队名>
```

最终文件在 `videos/`：

```text
预选赛赛段2任务1-仿真基础.mp4
预选赛赛段2任务2-关侧舱.mp4
预选赛赛段2任务3-起飞飞行.mp4
预选赛赛段2任务4-视觉对准.mp4
预选赛赛段2任务5-投放执行.mp4
预选赛赛段2任务6-精准度.mp4
```

## 5. 演示期间禁止事项

- 不同时启动 systemd Isaac 和手动 `scene_app.py`。
- 不在 PX4 运行后再用关门瞬态初始化 EKF。
- 不开启 `mission_autostart`。
- 不从 `trajectory_executor` 之外发布 `/fmu/in/*`。
- 不同时运行 PX4 和 `direct_rotor`。
- 不同时运行真实场景 cargo publisher 和 `cargo_status_sim`。
- 不重复运行 `ground_state_sim` 与 `mission_trigger`。
- 不为了赶截止时间扩大 prearm、导轨、落区或姿态安全门限。

## 6. 截止前打包

确认每段不超过 3 分钟、能播放、命名正确，然后：

```bash
bash docs/project/build_submission.sh <真实队名>
```

最终检查 zip、PDF、11 个必交视频和 `.md5`。赛段二已有完整物理闭环基线，录制时仍须
一镜到底并保留接口审计、rosbag、ULog 和落点测量；单轮通过不能写成 10 次重复验收。
