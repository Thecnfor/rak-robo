# /var/workspace/docker/isaac/workspace — ROS2 控制代码

colcon ROS2 workspace。**workspace/ 下面直接就是 src/，不要其他东西**（`.git/` 是 VCS 例外）。

## 目录约定

| 路径 | 是什么 | 注意 |
|---|---|---|
| `src/` | **colcon 源码目录**，所有 ROS2 packages 都在这里 | 写代码 |
| `.git/` | 版本控制（基础设施例外保留）| 你自己 commit |
| `.gitignore` | 忽略 build/install/log 等 | 见下 |

## 现有 packages

| 包 | 类型 | 用途 |
|---|---|---|
| `isaac_ros2_control/` | 你自己的 starter（ament_python）| cmd_vel_relay + tf_echo_bridge 两个 demo 节点，立刻能跑 |
| `grasp_demo_interfaces/` | 从原 demo_ws 拆出 | 抓取示例自定义 Action 接口 |
| `grasp_demo_pkg/` | 从原 demo_ws 拆出 | 抓取放置功能示例 |
| `nav2_demo_pkg/` | 从原 demo_ws 拆出 | 导航功能示例（AMCL/代价地图/规划）|
| `dual_arm_pkg/` | 比赛包（ament_python）| Mercury X1 双臂、双夹爪和地面抓取状态机 |
| `perception_competition_pkg/` | 比赛包（ament_python）| YOLOE、深度位姿估计和无人机下视对准 |
| `bridge_competition_pkg/` | 比赛包（ament_python）| host bringup、接口审计和 direct-rotor 烟测 |
| `drone_navigation_pkg/` | 比赛包（ament_cmake, GPL-3.0）| PX4 状态、局部规划、Offboard executor 和飞行状态机 |
| `competition_orchestrator_pkg/` | 比赛包（ament_python）| 空地协同、货舱门和无人机任务调度 |
| `px4_msgs/` / `px4_ros_com/` | 第三方 ROS 2 包 | PX4 消息、Offboard 示例和坐标系转换 |

## 无人机技术路线（2026-07-18 确认）

| 组件 | 状态 | 位置 / 责任 |
|---|---|---|
| **PX4 SITL** | ✅ 容器外部模拟器模式已运行；当前等待 Pegasus TCP 4560 | Docker 容器 `/var/workspace/docker/isaac/docker/px4/`，负责姿态 / 位置 / Offboard 闭环。完整用法见 `px4_sitl_usage.md` |
| **PegasusSimulator** | 提供 Multirotor 载具与原始传感器仿真，与 PX4 backend 联调 | Isaac Sim 内集成，发布 `/drone0/*`、点云和 TF |
| **EGO-Planner** | ✅ 滚动体素/动态 A*/B-spline 与无碰 polyline fallback 已完成单轮实景全流程；⚠️ 上游 raycast/LBFGS 逐文件来源票据仍待关闭 | `drone_navigation_pkg` 输出内部 trajectory，由唯一 executor 转为 PX4 Offboard；2026-07-31 已完成避障出航、分段返航和精准落架，正式重复验收仍需 10 次连续通过。见 [`../docs/setup/ego_planner_integration.md`](../docs/setup/ego_planner_integration.md) |

PX4 是唯一飞控，只有 `trajectory_executor` 可写 `/fmu/in/*`。四电机 topic 只在与 PX4 互斥的 direct-rotor 低速烟测模式使用。

详细环境方案见 [`../docs/setup/env_setup.md`](../docs/setup/env_setup.md)，包间契约见 [`../docs/contracts/interface_contracts.md`](../docs/contracts/interface_contracts.md)，运行步骤见 [`../docs/runbooks/drone_navigation.md`](../docs/runbooks/drone_navigation.md)。

## 工作流

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash

# 编 starter
colcon build --packages-select isaac_ros2_control
source install/setup.bash

# 跑
ros2 run isaac_ros2_control cmd_vel_relay
ros2 run isaac_ros2_control tf_echo_bridge

# 新建你自己的包
cd src
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy geometry_msgs
cd ..
colcon build --packages-select my_pkg
```

## ROS2 配置（**不要**手动设 environment）

`isaacsim51.service` 已经统一设好：
- `ROS_DOMAIN_ID=45`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISTRO=jazzy`
- `LD_LIBRARY_PATH=.../isaacsim.ros2.bridge/jazzy/lib`

host 端手动 export 同 4 个变量即可互通。在每个脚本里 export 反而容易踩坑。

## git

```bash
cd /var/workspace/docker/isaac/workspace
git add -A
git commit -m "init: isaac_ros2_control starter + 拆出的 demo packages"
```
