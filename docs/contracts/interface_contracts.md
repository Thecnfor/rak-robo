# 包间接口契约

> 2026-07-18 冻结。运行现场允许 ROS remap，但必须同步更新
> `bridge_competition_pkg/interface_audit.py`；接口审计报告是验收票据，文档中的
> “预期”不等于已在比赛场景验证。

## 唯一无人机控制链

```text
Isaac/Pegasus 传感器与动力学
  → PX4 SITL 状态估计与飞控
  → drone_navigation_pkg 规划/任务意图
  → trajectory_executor
  → /fmu/in/*
  → PX4 电机分配
  → Pegasus PX4MavlinkBackend
  → 四个电机
```

`trajectory_executor` 是以下三个 topic 的唯一发布者：

- `/fmu/in/offboard_control_mode` (`px4_msgs/OffboardControlMode`)
- `/fmu/in/trajectory_setpoint` (`px4_msgs/TrajectorySetpoint`)
- `/fmu/in/vehicle_command` (`px4_msgs/VehicleCommand`)

EGO、视觉、返航和业务 action 只能提交 `/drone/navigation/*` 控制意图。supervisor
是 canonical `goal/control_mode/cargo` 仲裁出口；orchestrator action 使用
`operator_goal/operator_mode`，禁止直接写 `/fmu/in/*`，也禁止在比赛主链写四旋翼转速。

## Isaac/Pegasus 原始接口

`px4` 模式下，`PX4MavlinkBackend` 排在第一个并独占电机输出；第二个
`ROS2Backend` 仅观测。预期原始输出如下：

| topic | type / 说明 |
|---|---|
| `/drone0/state/pose` | `geometry_msgs/PoseStamped`, ENU ground truth |
| `/drone0/state/twist` | `geometry_msgs/TwistStamped`, body velocity |
| `/drone0/state/twist_inertial` | `geometry_msgs/TwistStamped`, ENU velocity |
| `/drone0/state/accel` | `geometry_msgs/AccelStamped` |
| `/drone0/sensors/imu` | `sensor_msgs/Imu`, FRD/NED sensor data |
| `/drone0/sensors/mag` | `sensor_msgs/MagneticField` |
| `/drone0/sensors/gps` | `sensor_msgs/NavSatFix` |
| `/drone0/sensors/gps_twist` | `geometry_msgs/TwistStamped` |
| `/avoidance/lidar/pointcloud` | `sensor_msgs/PointCloud2`, frame `avoidance_lidar` |
| `/drone_0_ego_odom` | `nav_msgs/Odometry`, Isaac ENU 对照状态 |
| `/drone0/down_camera/color/image_raw` | `sensor_msgs/Image`, 640×480@30 Hz 下视相机 |
| `/cargo_bay/{command,status}` | `std_msgs/String` |

`direct_rotor` 只用于上锁/低速接口烟测，关闭 PX4 backend 后才允许写：

- `/drone0/control/rotor{0..3}/ref` (`std_msgs/Float64`, rad/s)
- 默认 Pegasus 旋向为 `[-1, -1, +1, +1]`；物理位置和 PX4 mixer 对应关系仍须由
  实际场景逐电机票据确认。

两个模式严格互斥，由环境变量 `DRONE_BACKEND=px4|direct_rotor` 选择。

## PX4 v1.16.2 接口

本工作区 `px4_msgs` 当前在 PX4 main 分支（v2.0.1），不是 `release/1.16`。
uXRCE-DDS 按 `px4_msgs/*.msg` 的 `MESSAGE_VERSION` 决定是否给 topic 加 `_vN` 后缀：
`VehicleStatus.msg` 的 `MESSAGE_VERSION=1` → 实际发布 `vehicle_status_v1`；
`VehicleOdometry/CommandAck/LandDetected` 都是 `MESSAGE_VERSION=0` → 未版本化。
运行时 `interface_audit.py::resolve_actual_topic` 自动把契约里的 unversioned 名
解析到带 `_v1` 的实际 topic。代码侧（`px4_state_adapter_node.cpp`、
`trajectory_executor_node.cpp`）直接订阅 `vehicle_status_v1`，不在 launch 层 remap。
| PX4 → host | host → PX4 |
|---|---|
| `/fmu/out/sensor_combined` | （只读估计器输入健康票据） |
| `/fmu/out/vehicle_odometry` | `/fmu/in/offboard_control_mode` |
| `/fmu/out/vehicle_status_v1` (MESSAGE_VERSION=1) | `/fmu/in/trajectory_setpoint` |
| `/fmu/out/vehicle_command_ack` | `/fmu/in/vehicle_command` |
| `/fmu/out/vehicle_land_detected` | 仅 executor 可写 |

PX4 输出订阅使用 `BEST_EFFORT/VOLATILE`。Offboard 以 20 Hz 发送，先预流至少
1 秒，再请求 Offboard，收到状态反馈后解锁。

## drone_navigation_pkg 公共接口

| 方向 | topic | type |
|---|---|---|
| 输入 | `/drone/navigation/mission_request` | `std_msgs/Bool` |
| supervisor → planner | `/drone/navigation/goal` | `geometry_msgs/PoseStamped`, frame `map` |
| supervisor → executor | `/drone/navigation/control_mode` | `std_msgs/String` |
| orchestrator → supervisor | `/drone/navigation/operator_{goal,mode}` | 公共 action 的仲裁请求 |
| 输入 | `/drone/drop_target_offset` | `[nx, ny, area_fraction, radius_px]` |
| 输出 | `/drone/navigation/odometry` | `nav_msgs/Odometry`, PX4 NED/FRD → map ENU/FLU |
| 输出 | `/drone/navigation/trajectory` | `drone_navigation_pkg/Trajectory` |
| 输出 | `/drone/navigation/planned_path` | `nav_msgs/Path` |
| 输出 | `/drone/navigation/state` | 任务状态机状态 |
| 输出 | `/drone/navigation/{planner_state,executor_state,px4_status,px4_command_ack}` | 诊断 |
| 输出 | `/drone/navigation/landed` | `std_msgs/Bool` |

状态机为：

```text
IDLE → PREFLIGHT → ARMING → TAKEOFF → EGO_TRANSIT → TARGET_SEARCH
→ VISUAL_ALIGN → DROP_HOLD → RETURN → LAND → COMPLETE
```

数据陈旧或 PX4 failsafe 进入 `HOLD`：0.60 秒悬停，1.20 秒请求 PX4 Land。
安全新鲜度使用 steady wall time；轨迹采样使用 `/clock`。

## 视觉、舱门和地空协调

- 视觉输入：`/drone0/down_camera/color/image_raw`
- 视觉输出：`/drone/drop_target_offset`、`/drone/drop_target_debug`、
  `/drone/drop_command`
- 舱门命令：`left_close|left_open|bottom_close|bottom_open`
- 舱门确认：`left_closed|left_opened|bottom_closed|bottom_opened payload_released`
- 地面完成门控接受：`COMPLETE`、`SUCCESS`、`GROUND_DONE`
- 总状态：`/arena/orchestrator/state`
- 公共 action：`/drone/flight_command` (`DroneFlightCommand`) 和
  `/cargo_bay/door_command` (`CargoDoorCommand`)
- 默认 `allow_manual_*_actions=false`：自主任务中拒绝 GOTO/RETURN/HOVER 和人工舱门命令；
  LAND/ABORT 始终经 supervisor 紧急 override 生效，避免 last-writer-wins。

## 运行时票据

```bash
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p report_path:=/tmp/drone_interface_report.json

# direct_rotor 模式另传：
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p backend_mode:=direct_rotor
```

报告必须记录 topic 类型、端点、QoS、观测频率、header frame/时间戳，并确认三个
`/fmu/in/*` 各自恰好只有 `/trajectory_executor` 一个发布者。当前未加载比赛场景时
只看到 `/cmd_vel`，因此在真实场景接入前不得把接口票据标为通过。
