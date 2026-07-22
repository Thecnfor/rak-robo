# 无人机全链路运行手册

## 当前交付状态

代码、消息、launch、Docker 外部模拟器模式、接口票据和可见无人机受控起降已经落地。
2026-07-22 的多次受控实飞完成 Offboard 预流、解锁、物理离架、HOLD、AUTO_LAND、落地和
自动解除武装；最新 `round1i` 使用全部安全修复后的构建并返回 `success=true`。正式验收仍需 1.8 m/30 s 悬停、速度收敛
调参、EGO 障碍/返航、投放区 `drop_search_pose` 和下视相机 PID。保持
`mission_autostart=false`，只通过安全探针分阶段放飞。

## 1. 环境

```bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

版本固定为 Isaac Sim 5.1、ROS 2 Jazzy、PX4 v1.16.2、`px4_msgs release/1.16`。

## 2. Isaac/Pegasus 模式

比赛主链：

```bash
export DRONE_BACKEND=px4
export ENABLE_PENCIL_PAYLOAD=1
export ENABLE_PREARM_SUPPORT=1
/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/run_demo_scene.sh --world X1
```

逐电机烟测：

```bash
export DRONE_BACKEND=direct_rotor
export ENABLE_PENCIL_PAYLOAD=1
export ENABLE_PREARM_SUPPORT=1
/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/run_demo_scene.sh --world X1
```

launcher 在 `env -i` 后显式透传上述两个开关。比赛默认均为 `1`；设为 `0` 只用于
故障隔离。起飞支架是桌面上的两个静态碰撞垫，不与无人机建立 joint，PX4 产生足够推力
后会自然脱离。`/cargo_bay/command=status` 应返回
`payload_locked=True prearm_support=True` 及载荷相对位姿误差。

不要同时运行 systemd 的 GUI Isaac 和上述独立 launcher。若 GUI 服务正在使用，应在
Kit Script Editor 执行：

```python
import os, runpy
os.environ["CARGO_DELIVERY_WORLD"] = "X1"
os.environ["DRONE_BACKEND"] = "px4"
runpy.run_path(
    "/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/scene_app.py",
    run_name="__main__",
)
```

也可由操作者明确停止 GUI 服务后再启动独立进程。场景内 `px4_autolaunch=false`，PX4
生命周期只归 Docker 管。

## 3. PX4 Docker

```bash
cd /var/workspace/docker/isaac/docker/px4
docker compose build
docker compose up -d
docker compose logs -f px4
```

Isaac 未加载时日志停在
`Waiting for simulator to accept connection on TCP port 4560` 是正确的安全状态；连接成功
后启动传感器、飞控和 uXRCE-DDS，届时 host 才会出现 `/fmu/*`。

独立 PX4 SIH 烟测与比赛主链分离：

```bash
PX4_SIM_MODEL=sihsim_quadx docker compose up -d --force-recreate
```

SIH 不能作为比赛飞行验收证据。

## 4. 接口票据

比赛场景播放、PX4 连接后执行：

```bash
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args \
  -p report_path:=/tmp/drone_interface_report.json \
  -p backend_mode:=px4
```

报告必须 `ok=true`；报告会记录实测频率、frame 和时间戳，还需人工核对电机映射。
direct 模式执行：

```bash
ros2 run bridge_competition_pkg direct_rotor_smoke_test \
  --ros-args -p enabled:=true -p rotor_speed_rad_s:=60.0
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p backend_mode:=direct_rotor
```

direct 测试只允许低速、机体固定/螺旋桨安全状态；程序结束会向四电机发零。完成后必须
重启场景回 `DRONE_BACKEND=px4`。

需要验证刚体确实响应单个电机时，使用独立的动态响应票据。支架校准完成后，可以先启动
`direct_rotor` 场景并确认稳定，再运行探针：

```bash
DRONE_BACKEND=direct_rotor ros2 run bridge_competition_pkg \
  direct_rotor_motion_probe --ros-args \
  -p enabled:=true -p rotor_index:=1 \
  -p rotor_speed_rad_s:=600.0 -p pulse_seconds:=1.0
```

探针必须输出 `passed=true`；无论成功或失败都会重复发送四电机零值。该票据最高使用
600 rad/s、1.0 秒的单电机动态脉冲，只能用于仿真且不得与 PX4 同时运行。当前支架
零电机 12 秒票据和非锁死响应数据见
`docs/project/prearm_support_evidence_2026-07-21.json`。

## 5. 标定参数

编辑 `src/drone_navigation_pkg/config/navigation.yaml` 后重建：

- `px4_map_origin`：PX4 local `(0,0,0)` 对应的 Isaac 世界 ENU 出生点；当前 X1 初值
  `[4.55,-0.38,1.13]`。
- `prearm_spawn_position`：Isaac 原始世界位姿的合法起飞支架中心，当前同为
  `[4.55,-0.38,1.13]`。预解锁还必须同时满足位置误差 ≤0.02 m、速度 ≤0.05 m/s、
  横滚/俯仰绝对值 ≤3°；支架状态字符串不能替代这项实体位姿检查。
- `drop_search_pose`：从最终 USD/比赛实景提取，当前 `[0,0,1.8]` 是占位。
- `takeoff_height` 和 `virtual_ceiling`：按净空校准。
- 相机必须确认 `/drone0/down_camera/color/image_raw` 是正下视；移动目标到图像四个方向，
  验证视觉速度符号后再调 `visual_kp`。
- `allow_manual_flight_actions` 和 `allow_manual_door_actions` 比赛默认必须为 `false`；只有
  独立人工接口测试才临时开启，LAND/ABORT 不受该开关限制。

## 6. Host bringup

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
```

bringup 启动四个导航节点、视觉检测、地空协调和接口审计；Foxglove（8765）由独立
systemd user service 常驻。默认不会自动解锁；只有 `/arena/ground/state` 为
`COMPLETE|SUCCESS|GROUND_DONE` 后 orchestrator 才发一次任务请求。该状态发布者和
两个消费者统一使用 `RELIABLE + TRANSIENT_LOCAL`，确保后启动的空中链也能收到已完成
状态；若运行时端点显示 `VOLATILE`，不得进入自动任务。

### Foxglove 3D 只看无人机

3D 面板出现大量 `left_gripper_*`、`wheel_link` 文本和重叠坐标轴，是 Foxglove 把整台
X1 的 `/tf` frame 名称全部显示出来，不代表无人机模型或 TF 树散架。面板按以下方式配置：

1. `Fixed frame` 设为 `map`，跟随 frame 设为 `avoidance_base_link`。
2. 在 `Transforms` 中关闭全局 `Labels` 和 `Axes`；需要核对时只临时打开
   `avoidance_base_link`、`avoidance_lidar`。
3. 只保留 `/avoidance/lidar/pointcloud`、`/drone/navigation/planned_path` 和无人机
   odometry/pose，隐藏地面机器人 `RobotState`/TF 可视项。

这样只改变显示层，不修改 ROS topic、TF 发布者或飞控坐标系。

## 7. 分阶段放飞

严格按下列顺序，每项保存 rosbag/PX4 日志：

1. 未解锁：传感器、坐标和接口报告。
2. Offboard：预流、切模式、解锁、1.8 m 起飞、30 s 定点、Land。
3. 单目标：0.5 m/s 到空旷目标并返航。
4. 障碍：静态、新出现障碍、局部死路。
5. 视觉：粗搜索位、稳定 0.8 s、水平速度 <0.05 m/s、开底舱。
6. 完整任务：关侧门→起飞→避障→对准→投放→返航→降落。

实测 RTX 负载下 PX4 odometry wall-time 间隔 p99 为 0.38 s，因此当前超过 0.60 s 进入
HOLD，超过 1.20 s 或 PX4 failsafe 请求 Land。轨迹进度和试飞阶段超时使用 `/clock`；
传输失联保护使用 steady wall time。

`LAND`/`ABORT` 会锁存终端降落状态；`CLEAR` 只清除普通操作意图，不解除该安全锁。
确认 PX4 已落地并解除武装后，必须显式发送 `RESET` 才能开始下一轮。
安全探针会在每轮开始自动执行这项 RESET 握手，并要求 executor=`DISABLED`、PX4
`AUTO_LOITER`、未解锁及支架上静稳后才进入 preflight。每次落地偏离桌面后必须重载
场景，直到 `/drone/navigation/state` 报告 `prearm_pose_valid=true`；不得通过扩大容差
绕过。同时必须看到 `planner_map_ready=true`，且 planner state 内的 `map_age`、`tf_age`
不超过 0.60 s；只有点云频率、没有成功 TF/voxel 更新不能作为地图就绪证据。只验证
预检而不解锁时执行：

```bash
ros2 run drone_navigation_pkg px4_hover_probe --ros-args \
  -p preflight_only:=true \
  -p output_path:=/tmp/prearm_support_acceptance.json
```

固定设定点诊断默认关闭，只有定位 PX4/机体模型问题时显式开启；它仍通过 supervisor
提交意图，`trajectory_executor` 保持 `/fmu/in/*` 唯一写入者：

```bash
ros2 launch drone_navigation_pkg navigation.launch.py \
  allow_fixed_setpoint_diagnostic:=true
ros2 run drone_navigation_pkg px4_hover_probe --ros-args \
  -p preflight_only:=false \
  -p fixed_setpoint_diagnostic:=true \
  -p fixed_step_clearances:="[0.10]" \
  -p fixed_hold_altitude:=1.23
```

探针把 `/clock` 的 wall-time 失联阈值独立设为 5 s，其余原始位姿、PX4 传感器和
里程计仍为 1.5 s；固定阶梯若水平偏移 >0.20 m、低于 home >0.10 m、速度 >0.5 m/s
或倾角 >20°，会先尝试返回落区再 LAND。中断或降落确认延迟时，进程持续发布安全
意图直到 PX4 确认 disarm，禁止在 `ACTIVE` 状态直接退出。2026-07-22 的四轮证据见
`docs/project/drone_fixed_setpoint_evidence_2026-07-22.json`；当前不得继续放飞，须先
把窄支架改成无 joint 的横向限位起飞托架并重做 +0.10 m 票据。

现场的拒绝/通过对照数据见
`docs/project/prearm_pose_gate_evidence_2026-07-22.json`。

## 8. 验收证据

- 10 次连续完整流程，无碰撞、坠机、重置或人工干预。
- 投放至少 90% 静止落点距圆心 ≤0.2 m。
- 保存 `/tmp/drone_interface_report.json`、rosbag、ULog、状态机日志、规划轨迹和全程视频。
- 接口/标定不通过时，不能以“节点已启动”替代比赛验收。
- 当前受控起降基线见
  `docs/project/drone_hover_evidence_2026-07-22.json`；它证明“真实能飞”，不替代正式
  1.8 m/30 s 和 10 次连续完整流程验收。
