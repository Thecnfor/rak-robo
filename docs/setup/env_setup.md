# 环境安装 · 统一方案

> 2026-07-18 更新：PX4 v1.16.2 Docker 外部模拟器模式已部署；只有 Pegasus 接入 TCP 4560 后才会启动完整飞控/uXRCE 数据流。EGO 不部署上游 ROS 图，规划和 PX4 适配统一在 GPL-3.0 的 `drone_navigation_pkg`，bridge 只负责传输、bringup 和接口审计。

## ✅ 装好的

| 项 | 版本/位置 | 备注 |
|---|---|---|
| PegasusSimulator | `~/Documents/PegasusSimulator` | 5.1 兼容，提供 Multirotor vehicle + 3 个 backend |
| PX4 SITL | `/var/workspace/docker/isaac/docker/px4/` | Docker 部署，PX4 v1.16.2，通过 uXRCE-DDS/MAVLink 联调 |
| px4_msgs / px4_ros_com | `workspace/src/` | ROS 2 与 PX4 的消息、Offboard 控制及坐标系转换 |
| cuRobo | 0.8.0 | Isaac Sim 5.1 依赖版本已回退 |
| Isaac Sim | 5.1.0 (conda `isaacsim51`) | ROS_DOMAIN_ID=45, RMW=`rmw_fastrtps_cpp` |
| ROS2 Jazzy | `/opt/ros/jazzy` | 全套装好 |

## 技术路线

| 组件 | 使用方式 |
|---|---|
| PX4 SITL | ✅ 已部署。Docker 容器跑 PX4 v1.16.2 SITL + uXRCE-DDS agent；外部仿真器连接后在 Domain 45 发布未版本化 `/fmu/*`。负责姿态、位置和 Offboard 闭环 |
| PegasusSimulator | 提供 Isaac Sim 5.1 Multirotor、动力学和原始传感器；主链由 PX4 backend 独占电机，observer 发 `/drone0/*`，场景另发点云/TF |
| EGO-Planner | ⚠️ `drone_navigation_pkg` 内已有隔离的滚动体素/A*/B-spline 安全占位核心；**不部署上游 ROS 图**。指定 commit 不存在于官方 remote，raycast/LBFGS 移植尚未完成，见 [`ego_planner_integration.md`](ego_planner_integration.md) |
| 简单 LiDAR 反应式避障 | 作为 EGO 算法集成前的烟测与故障降级路径，不是最终主规划器 |
| ArduPilot SITL | 不采用；飞控统一使用 PX4 |

## 飞控链路：PX4 SITL

```text
competition_orchestrator_pkg / drone_navigation_pkg
  -> PX4 OffboardControlMode + TrajectorySetpoint + VehicleCommand
  -> /fmu/in/*
  -> PX4 SITL
  -> /fmu/out/vehicle_odometry 等状态
  -> PegasusSimulator / Isaac Sim 载具与传感器
```

具体启动、QoS 和 `/fmu/*` 话题说明见 `src/px4_sitl_usage.md`。`/drone/cmd_vel` 可作为项目内部业务指令，但在进入 PX4 前必须由适配节点转换为 PX4 Offboard 设定点，不直接驱动旋翼关节。

## 轨迹规划：复用 EGO-Planner 算法库

```text
/avoidance/lidar/pointcloud + /drone/navigation/odometry + /drone/navigation/goal
  -> ego_local_planner (drone_navigation_pkg)
     ├─ voxel map + dynamic A*
     ├─ Uniform B-spline + feasibility scaling
     └─ trajectory_executor @20 Hz
  -> /fmu/in/trajectory_setpoint
  -> /fmu/in/offboard_control_mode (心跳)
  -> /fmu/in/vehicle_command (ARM / set_mode / takeoff / land)
```

不依赖 EGO-Planner 上游 ROS 2 package 的 launch、topic 命名、Humble/Jazzy 工程配置或 RViz 插件。纯 C++ 核心由 `flight_core` 暴露深接口，ROS 2 包装与 PX4 Offboard 转换在同一包的节点中完成。完整契约见 [`../contracts/interface_contracts.md`](../contracts/interface_contracts.md)。

---

## 验证清单

```bash
source /home/socl/miniconda3/bin/activate isaacsim51

# cuRobo
python -c "import curobo, torch; assert torch.cuda.is_available(); print('curobo OK', curobo.__version__, 'on', torch.cuda.get_device_name(0))"

# PegasusSimulator（Kit Script Editor）
python -c "from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend; print('pegasus ROS2 backend OK')"

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash
ros2 topic list   # 应该看到 /clock /parameter_events /rosout

# PX4 SITL + DDS
cd /var/workspace/docker/isaac/docker/px4
docker compose up -d
docker compose logs -f px4   # Ready for takeoff + time sync converged

# host 端 PX4 话题
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic list | grep /fmu/
```

## 已知坑（已经踩过的）

1. **PyPI 上的 `curobo` 0.2 是假的同名包**——必须从 GitHub 装 NVIDIA 版
2. **cuRobo 装完会升级 websockets/packaging**——立即 `--force-reinstall websockets==12.0 packaging==23.0` 回退
3. **PegasusSimulator 路径硬编码 `/home/robot-a/`**——必须改成 `/home/socl/Documents/PegasusSimulator/`
4. **不要在 Ubuntu 24.04 host 上继续编译旧 PX4 v1.14**——项目使用 Docker 中的 PX4 v1.16.2，避免 GCC 13 兼容问题
5. **不要直接部署 EGO-Planner 的 ROS 2 工程**——规划核心和 ROS 2 适配已由 `drone_navigation_pkg` 统一封装；bridge 不拥有规划算法
6. **PX4 ROS 2 topic 的 QoS 不是默认可靠模式**——调试 `/fmu/*` 时按 `src/px4_sitl_usage.md` 使用匹配的 BEST_EFFORT/TRANSIENT_LOCAL 配置
7. **PX4 用 NED/FRD，ROS 用 ENU/FLU**——向 PX4 发送向量用 `(x_e,y_n,z_u)→(y_n,x_e,-z_u)`；姿态由 `px4_state_adapter` 的单元测试覆盖，不要在业务节点重复手写

## 备份脚本（D 维护，cron 跑）

```bash
# /home/socl/backup_env.sh —— 每晚 23:00 备份 conda env
conda-pack -n isaacsim51 -o /var/workspace/backups/isaacsim51_$(date +%Y%m%d).tar.gz
```

装 conda-pack：`conda install -c conda-forge conda-pack`
