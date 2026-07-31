# 无人机全链路运行手册

## 当前交付状态

代码、消息、launch、Docker 外部模拟器模式、接口票据和无人机比赛流程已经落地。
2026-07-31 的最新冷启动实景票据连续完成关侧舱、Offboard 解锁起飞、EGO 避障、
下视视觉对准、开底舱投放、经通道点返航、毫米级落架、PX4 Land、判地和自动解锁，
最终状态为 `COMPLETE`。rosbag
`bags/stage2_fullflow_20260731_1500` 记录了 2561.6 s 到 `COMPLETE` 的状态链；静止
投放误差 1.27 cm，远离 home 的返航最低高度 1.772 m，Land 交接水平误差约
5.46 mm、速度 0.0499 m/s，最终速度约 0.00010 m/s，全程没有 PX4 failsafe。
接口报告 `/tmp/stage2_interface_1500.json` 为 `ok=true` 且 `/fmu/in/*`
保持唯一写入者。
可提交的指标摘要见
`docs/project/drone_stage2_fullflow_evidence_2026-07-31.json`。

该轮三次导航输入短暂超时都自动 HOLD/恢复；最后一次暴露了 RETURN 的 waypoint
显示 latch 在 HOLD 恢复时被清零，以及 BEST_EFFORT volatile 离散状态可能错过最后
`landed=true`。飞机实际始终保持 `RETURN_FINE`，PX4 原始状态最终为
`landed=true/armed=false`。落地后已修成 HOLD→RETURN 不重置 latch，并把
VehicleStatus/LandDetected 订阅改为 transient-local；59 项核心测试和地面实时
`/drone/navigation/landed=true` 验证通过。两项收尾改动尚未再跑一轮完整飞行，
因此仍需计入下一次连续回归。

这是一条“PX4 主飞控 + Isaac 物理真值安全见证”的仿真基线：航路规划和常规轨迹跟踪
使用 PX4 估计里程计，但起飞导轨 XY 接管时机、预解锁姿态一致性和返架最后
`0.45 m` 的低速修正读取 `/drone0/state/pose`。它不是纯 PX4 估计闭环。提交前必须
确认比赛允许使用该场景原始状态接口；若不允许，应以允许的视觉/测距定位替代近场修正，
不得把真值依赖隐藏为“仅诊断”。本轮记录的真值速度峰值为 `0.606 m/s`，高于
`max_velocity=0.3 m/s` 的规划值；该值包含 PX4 物理跟踪瞬态，须在连续回归中确认
不越过最终比赛安全包线。

2026-07-26 的完整高度票据完成 Offboard 预流、解锁、七级爬升至世界高度
1.80 m、5 s HOLD、RETURN_HOME、AUTO_LAND、落架和自动解除武装。返航用 Isaac
真值外环进入 8 mm 捕获圈，最终触地点距托架中心约 10.2 mm，位于单侧 10 mm
导轨间隙和 ±20 mm 承重区内。ULog 实际最大横滚/俯仰为 0.289°/0.694°，四电机
峰值为 0.584--0.602，无饱和、failsafe 或 failure-detector 标志。随后正式
1.80 m/30 s HOLD 也通过：最大速度 0.0267 m/s、高度误差 15.4 mm，ULog 最大实际
横滚/俯仰 0.240°/0.888°、电机峰值 0.504--0.518。当前单轮完整比赛流程已通过，
正式验收仍需完成 10 次连续无碰撞回归和至少 10 次投放统计，确认 90% 落点误差
不超过 0.2 m。保持 `mission_autostart=false`，由地面完成信号触发任务。

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

PX4 外部仿真必须按以下顺序初始化，不能让 EKF 在侧舱门关闭瞬态中建立姿态原点：

1. `docker compose down` 停止 PX4，再加载并播放 Isaac 场景。
2. 依次向 `/cargo_bay/command` 发布 `left_close`、`bottom_close`。
3. 确认 `/drone0/state/pose` 连续 3 s 处于
   `(4.55,-0.38,1.13)±0.004 m`、速度 `<0.05 m/s`、横滚/俯仰 `<0.5°`。
4. 最后启动 PX4 Docker，等 `/fmu/out/vehicle_status_v1` 报告预检通过。

2026-07-23 的对照试验证明，场景默认 `left_open()` 后立即启动 PX4 会把关门瞬态带入
EKF；按上述顺序启动后，Isaac/PX4 横滚和俯仰差缩小到约 `0.03°`。
预解锁 `0.004 m` 位置门限用于固定出生点和 PX4 估计原点。当前导轨单侧物理间隙为
`0.010 m`；返航进入门限为 `0.008 m`，保留 `2 mm` 机械余量。触地点落在导轨
角部时即使旧的 `0.02 m` 落区门限会通过，也必须重载场景重新居中，不能直接进行
下一轮增推。

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
  `[4.55,-0.38,1.13]`。预解锁还必须同时满足位置误差 ≤0.004 m、速度 ≤0.05 m/s、
  横滚/俯仰绝对值 ≤3°；支架状态字符串不能替代这项实体位姿检查。
- `drop_search_pose`：当前 X1 圆心标定为 `[5.5,-3.5,1.8]`。
- `return_transit_waypoint`：返航穿越长隔断的安全通道点，当前
  `[5.5,-1.7,1.8]`；抵达后单向锁存，不因漂移回退。
- `return_approach_pose`：EGO 在起飞托架膨胀区外的终点，当前
  `[4.55,-0.75,1.8]`。进入 `return_fine_radius=0.45 m` 后由限速近场环先横向对中，
  进入 `return_descent_radius=0.02 m` 后才下降。
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

实测 RTX 负载下 PX4 odometry wall-time 间隔 p99 为 0.38 s；2026-07-23 的飞行录包
另捕获一次 1.372 s 尾延迟。因此当前超过 0.60 s 进入 HOLD，超过 1.50 s 或 PX4
failsafe 请求 Land。轨迹进度和试飞阶段超时使用 `/clock`；
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

`landing_region_verified` 默认是 `false`，因此探针会报告落区门未通过并禁止解锁。
只有从当前 USD 桌面/托架世界包围盒量出落区、确认矩形完全位于可承重表面后，才能
同时传入 `landing_region_center`、`landing_half_extents` 并设置
`-p landing_region_verified:=true`；不得用实时机体位置自动重设落区中心。
固定设定点探针在正常返回和可恢复异常返回时，都会以该落区中心作为 LAND 前的
水平目标；因此起飞托架可以与安全降落区分离。

固定设定点诊断默认关闭，只有定位 PX4/机体模型问题时显式开启；它仍通过 supervisor
提交意图，`trajectory_executor` 保持 `/fmu/in/*` 唯一写入者：

```bash
ros2 launch drone_navigation_pkg navigation.launch.py \
  allow_fixed_setpoint_diagnostic:=true \
  fixed_vertical_only_diagnostic:=true
ros2 run drone_navigation_pkg px4_hover_probe --ros-args \
  -p preflight_only:=false \
  -p fixed_setpoint_diagnostic:=true \
  -p landing_region_center:="[4.5575,-0.3802]" \
  -p landing_half_extents:="[0.02,0.02]" \
  -p landing_region_verified:=true \
  -p fixed_step_clearances:="[0.10]" \
  -p fixed_hold_altitude:=1.2309 \
  -p fixed_step_timeout:=30.0 \
  -p prearm_position_tolerance:=0.004 \
  -p fixed_target_horizontal_tolerance:=0.02 \
  -p fixed_target_altitude_tolerance:=0.015 \
  -p cradle_touchdown:=true \
  -p cradle_touchdown_horizontal_tolerance:=0.008 \
  -p cradle_approach_clearance:=0.055 \
  -p landing_truth_xy_gain:=2.0
```

探针把 `/clock` 的 wall-time 失联阈值独立设为 5 s，其余原始位姿、PX4 传感器和
里程计仍为 1.5 s。executor 明确报告 `fixed_vertical_active=false` 前，固定阶梯水平
偏移超过 `0.012 m` 会直接 LAND。完整
XY 控制接管后水平门限恢复为 `0.20 m`，此时
才允许先返回落区再 LAND。低于 home >0.10 m、速度 >0.5 m/s 或倾角 >20°同样判定
异常。验收 HOLD 期间任一采样若 XY 误差 >0.10 m、
高度误差 >0.05 m、速度 >0.05 m/s 或归一化电机输出达到 0.95，整轮判失败；解除锁定
若现场另行开放只读 `/fmu/out/actuator_motors` 并设置
`require_live_actuator_feedback=true`，HOLD 中反馈超过 0.30 s 未更新同样失败。当前 PX4
v1.16 默认 DDS 不开放该输出，故每次飞行必须从 ULog 复核饱和度；未完成 ULog 分析时
探针不会把飞行标记为验收成功。解除锁定后仍用最终实测触地点复核落区。中断或降落
确认延迟时，进程持续发布安全
意图直到 PX4 确认 disarm，禁止在 `ACTIVE` 状态直接退出。横向限位托架模式必须同时
启用 `fixed_vertical_only_diagnostic`：首飞只锁高度，XY 使用零加速度，航向角留空并
命令零偏航速率，避免机体尚受限时由估计漂移积累位置/航向控制量。当前物理导轨直接
包围 `/World/quadrotor/body/body_collision`，单侧间隙为 `10 mm`，顶部为
`+0.05 m`。当前实飞配置在上升超过 `+0.005 m` 后接管完整 XY/航向控制，并在
下降到 `+0.003 m` 内恢复垂直模式；该数值以
`navigation.yaml` 的 `fixed_vertical_release_clearance`/
`fixed_vertical_reengage_clearance` 为准，不再沿用旧文档的 `+0.03/+0.02 m`。
不满足配置约束时节点拒绝启动。请求 Land 前 Isaac 真值水平偏移还必须小于
`8 mm`，给当前导轨
保留 `2 mm` 机械余量；prearm 出生点容差仍为独立的 `4 mm`。接管时捕获 PX4 当前估计航向，
避免航向设定点阶跃。探针默认在偏航率超过
0.5 rad/s 时直接请求 PX4 Land；`cradle_touchdown` 会在 LAND 前先用同一模式垂直
回到支撑面上方。实飞将 Land 交接高度从 0.08 m 收紧为 0.055 m；继续降低会让完整
XY 控制深入 0.05 m 高的物理导轨，未经新的接触票据不得采用。

2026-07-22 的 +0.10 m 票据已通过：最高离架 0.1171 m、HOLD 最大速度 0.0381 m/s、
触地点距验证中心 0.01243 m，四电机 ULog 峰值为 0.5345--0.5364，0 个采样达到
0.95 饱和门限，5 个 PX4 命令 ACK 全部成功。证据见
`docs/project/drone_cradle_takeoff_evidence_2026-07-22.json`。这只解除托架内 +0.10 m
首飞票据，不替代 +0.20 m/10 s HOLD、自由飞行或完整比赛流程验收。

2026-07-23 的短导轨交接修复也已实飞通过：XY 在 PX4/raw 离架
`0.0403/0.0442 m` 时接管，水平位移仅 `0.0073 m`；5 秒 HOLD 最大水平误差
`0.0215 m`、高度误差 `0.0222 m`、速度 `0.018 m/s`，返架后触地点距验证中心
`0.0173 m`。ULog 四电机峰值为 `0.5006--0.5132`，没有饱和、failsafe 或 EKF
故障。证据见
`docs/project/drone_low_guide_handoff_evidence_2026-07-23.json`。下一阶段才允许提高到
`+0.20 m/10 s`，不得直接跳到 1.8 m。

同日的首次 `+0.20 m` 试验因俯仰振荡在 `20.4°/0.219 m/s` 触发安全中止；飞机在
桌面完成 Land 和 disarm。ULog 显示俯仰设定最大约 `5.9°`、实际达到 `21.6°`，且无
电机饱和，因此当前开放问题是姿态/角速度动态或仿真惯量匹配。随后临时把
`MC_ROLLRATE_K/MC_PITCHRATE_K` 从 `1.0` 降到 `0.5` 的对照试验会在交接前横滑，
最终滑出台面，已经判定不安全并从 Docker Compose 回退。禁止复用 `K=0.5`；再次通电
前必须重载场景扶正无人机，先核对 PhysX 质量/惯量与 PX4 模型，再保持 rate K=1.0
逐项验证外环姿态参数。该失败也证明旧的导轨内 `0.05 m` 横移门限晚于物理边界；现已
收紧为 `0.012 m`。这是历史失败结论；2026-07-26 的新 +0.10 m 票据通过后已重新
授权下一项 +0.20 m/10 s 测试。

随后三次 `MC_PITCH_P=4.0`、rate K 保持 `1.0` 的 `+0.10 m` 诊断均由安全门限主动
LAND 并完成 disarm，没有坠机。ULog 实际最大俯仰分别约 `1.66°`、`0.82°`，最后一轮
最大横滚约 `1.16°`，电机峰值低于 `0.557`；因此这些轮次没有复现 +0.20 m 的姿态
发散。旧导轨却引用了无碰撞 API 的 `transparent_cargo_bay` 外观包围盒，真实
`body_collision` 单侧间隙仍为 15--17.5 mm，12 mm 门限会在产生导轨接触前触发。
场景先改为用实际主刚体碰撞盒生成导轨，随后把单侧间隙校准为 10 mm，并要求 PX4
启动前关闭左侧舱门。
由于货舱在 Y 方向比主机身更宽，四面导轨还必须显式过滤货舱本体、两扇舱门、锁定
载荷和旋翼，只保留与主机身碰撞；否则初始重叠会令 PhysX articulation 变成 NaN。
该过滤修复已通过 9 项离线几何测试；零电机实景连续 10 秒收到 136 个采样，保留的
每秒样本最大位置漂移为 `1.23e-7 m`、横滚/俯仰为 0°、最大速度为
`0.000139 m/s`，且服务跨过旧故障时间点后仍保持
active。该修复此后已由 2026-07-26 的 +0.10 m 带电起降票据关闭；+0.20 m 必须继续
沿用同一 5°/0.15 m/s/电机 0.95 安全包线。

`fixed_guided_horizontal_limit_m=0.012` 是“导轨接触失效时立即 LAND”的独立软件
故障门限，不是导轨自由间隙；不得再要求它小于 4 mm 的 prearm/返架居中门限。

2026-07-26 后续阶梯已经取代上述“下一次”限制：单轮
`+0.10/+0.20/+0.30 m`、2 s HOLD 和返架通过；随后单轮
`+0.10/+0.20/+0.30/+0.40/+0.50/+0.60/+0.67 m` 到世界高度 1.80 m、5 s HOLD、
返架、Land、disarm 全部通过。完整高度票据为
`/tmp/drone_stage2_fullheight_180m_return_20260726.json`，ULog 为
`docker/px4/ulog/2026-07-26/12_57_18.ulg`。所有 5 个 PX4 命令 ACK 成功；HOLD
最大高度误差 8.9 mm、最大速度 0.0319 m/s，返航最小 Isaac 真值水平误差
0.71 mm。ULog 静态返航代理因无法观测 Isaac 真值外环而不作为毫米级落架的最终
判据；其职责仅是复核 PX4 估计侧轨迹。实景 raw pose、落区包含关系和 ULog
姿态/电机/failsafe 三类证据必须同时保存。

随后 `1.80 m/30 s HOLD` 也已通过，组合证据见
`docs/project/drone_fullheight_hold30_evidence_2026-07-26.json`。因此当前开放前沿
已由 2026-07-31 的完整流程票据推进为连续回归；单轮通过不得写成“10 次验收完成”。

以下是 2026-07-23 的历史恢复步骤，保留用于故障回归，不再代表当前开放高度：

1. 在无 PX4 状态下验证六个支撑目标存在、主刚体到四面导轨的间隙为 `5±1 mm`，并用
   PhysX 接触报告或低速水平推压证明 `body_collision` 会碰到导轨。
2. 启动 PX4 后先完成静置姿态门限，确认 raw/PX4 横滚俯仰差均小于 `3°`。
3. 仅运行 `+0.10 m`、5 s HOLD；要求交接前水平偏移 `<0.012 m`、交接发生在
   `+0.03 m`，全程倾角 `<15°`、速度 `<0.05 m/s`，并自动返架、LAND、disarm。
4. 首次部署先安装锁定版本的 ULog 工具依赖：
   `python3 -m pip install --user -r src/drone_navigation_pkg/requirements-flight-tools.txt`。
   随后对 ULog 运行
   `ros2 run drone_navigation_pkg ulog_attitude_audit <file.ulg>`；只有探针、ULog、
   接触证据三者同时通过后，才重新开放 `+0.20 m/10 s`。

本轮数据摘要见
`docs/project/drone_pitch_handoff_evidence_2026-07-23.json`；零电机原始抽样见
`docs/project/drone_prearm_static_stability_2026-07-23.json`。

现场的拒绝/通过对照数据见
`docs/project/prearm_pose_gate_evidence_2026-07-22.json`。

## 8. 验收证据

- 10 次连续完整流程，无碰撞、坠机、重置或人工干预。
- 投放至少 90% 静止落点距圆心 ≤0.2 m。
- 保存 `/tmp/drone_interface_report.json`、rosbag、ULog、状态机日志、规划轨迹和全程视频。
- 接口/标定不通过时，不能以“节点已启动”替代比赛验收。
- 通过新出现障碍的动态换轨回归：折线后备轨迹已在每个拐角停车并满足速度/加速度
  限制；普通 ACTIVE 路径仍锁到端点以避免零速段重复重启，后续需增加显式
  `ACTIVE_OBSTACLE_REPLAN` 抢占后再做实景障碍测试。
- 确认 `/drone0/state/pose` 的比赛合规性，或完成允许传感器的近场定位替换。
- 记录每轮真值速度峰值；正式安全上限确认前，`0.606 m/s` 单轮峰值不得当作已调参完成。
- 2026-07-31 单轮完整流程基线：
  `bags/stage2_fullflow_20260731_1500`、
  `/tmp/stage2_interface_1500.json` 和
  `docs/project/drone_stage2_fullflow_evidence_2026-07-31.json`。状态序列实际达到
  `IDLE → PREFLIGHT → ARMING → TAKEOFF → EGO_TRANSIT → TARGET_SEARCH →
  VISUAL_ALIGN → DROP_HOLD → RETURN → LAND → COMPLETE`；两次短暂传感器 HOLD
  均自动恢复，不属于人工干预。
- 当前受控起降基线见
  `docs/project/drone_hover_evidence_2026-07-22.json`；它证明“真实能飞”，不替代正式
  1.8 m/30 s 和 10 次连续完整流程验收。
