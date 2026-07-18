# PX4 v1.16.2 Docker 使用指南

容器目录：`/var/workspace/docker/isaac/docker/px4`。

## 比赛主链启动

```bash
cd /var/workspace/docker/isaac/docker/px4
docker compose build
docker compose up -d
docker compose logs -f px4
```

默认 `PX4_SIM_MODEL=gazebo-classic_iris`，PX4 作为外部模拟器客户端连接
Pegasus TCP 4560。Isaac 场景未加载时，日志停在
`Waiting for simulator to accept connection` 是安全且正常的；此时不能期待 `/fmu/*`
已经出现。

SIH 只用于独立 PX4 烟测，不进入比赛链：

```bash
PX4_SIM_MODEL=sihsim_quadx docker compose up -d --force-recreate
```

## DDS 验证

Pegasus 连接成功后：

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic list | sort | grep '^/fmu/'
ros2 topic info -v /fmu/out/vehicle_status
ros2 topic echo --qos-reliability best_effort /fmu/out/vehicle_status --once
```

本项目 PX4 v1.16.2 使用未版本化 `/fmu/out/vehicle_status`，不要再写
`vehicle_status_v1`。工作区 `src/px4_msgs` 必须保持 `release/1.16`。

导航主链依赖：

- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_command_ack`
- `/fmu/out/vehicle_land_detected`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

最后三个 topic 只能由 `drone_navigation_pkg/trajectory_executor` 发布。它以 20 Hz
发送，先预流至少 1 秒，再请求 Offboard，收到状态后才解锁。

## 数据路径

```text
Pegasus sensors/dynamics
  ↔ Simulator MAVLink TCP 4560
PX4 SITL
  ↔ uXRCE client UDP 8888
MicroXRCEAgent
  ↔ Fast DDS Domain 45
host ROS 2 Jazzy
```

## 诊断

```bash
# 容器和 PX4 日志
docker compose ps
docker compose logs --tail=200 px4

# agent
docker exec px4-sitl ps -eo pid,state,comm | grep MicroXRCE
ss -ulnp | grep ':8888'
tail -n 100 /var/workspace/docker/isaac/docker/px4/logs/agent.log

# 接口票据（场景连接后）
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p report_path:=/tmp/drone_interface_report.json
```

| 现象 | 判断 |
|---|---|
| 一直等待 TCP 4560 | Isaac/Pegasus 比赛场景尚未运行或端口未接入 |
| 端点存在但无数据 | 检查 Agent、Domain 45、Fast DDS 和 BEST_EFFORT QoS |
| `invalid message type` | 重新构建/source `px4_msgs release/1.16` |
| Offboard 立即退出 | 检查 20 Hz 心跳、1 秒预流、唯一写入者和 PX4 failsafe |

ULog 位于容器
`/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`（以实际启动日志为准），
Agent/rosbag 持久化目录分别是 Docker 目录下的 `logs/`、`bags/`。
