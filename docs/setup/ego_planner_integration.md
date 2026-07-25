# EGO 规划集成：只用算法，不部署上游 ROS 图

## 冻结决策（2026-07-20 close-out）

- 请求的上游基线 `ZJU-FAST-Lab/ego-planner` commit `23a8d5a191…` 在官方 remote
  上 `not our ref`，2026-07-20 决定用本地最小核心**替掉**此 ticket
  （用户裁决）。不取得可达 fork，不再回追。
- `drone_navigation_pkg/flight_core` 的滚动体素 / 26 邻域 A* / 三次 B-spline
  + 可行性拉伸是比赛主链规划器，**不再视为占位**。
- 比赛文档 / 视频 / 技术报告里讲清：项目不复用 EGO 上游源码，自研最小核心
  是最终方案，避免评委质疑"项目是不是空跑占位"。
- 旧 ticket（"取得指定上游 commit" / "移植 raycast/LBFGS"）从
  `docs/project/tasks_full.md` 移除。

## 原始决策（2026-07-18 冻结，已废止）

- 请求的上游基线：`ZJU-FAST-Lab/ego-planner` commit
  `23a8d5a191711dd65633df689bd00f55d4dea8f9`，GPL-3.0。2026-07-18
  对官方 remote 的直接 fetch 结果为 `not our ref`；在取得该 ROS 2 fork 的可达 remote
  前，不能把当前实现标记为"已复用指定 commit"。
- 不启动上游 launch、swarm、SO3 controller、模拟器、topic 或 TF。
- 算法和 ROS/PX4 边界统一放在 GPL-3.0 的 `drone_navigation_pkg`。
- `bridge_competition_pkg` 只负责 bringup、接口审计和 direct-rotor 烟测。
- `/fmu/in/*` 只能由 `trajectory_executor` 写。

这修正了旧文档中"EGO 放进 bridge""`vehicle_status_v1`""1 Hz 心跳"和
":EGO 直接出电机转速"等偏差。

## 当前实现边界

`drone_navigation_pkg` 已实现一个与 EGO 架构同向、隔离于 ROS 的规划核心：

| 模块 | 当前实现 |
|---|---|
| 局部地图 | 0.10 m 滚动体素桶、1.0 s 衰减、有限点过滤、0.25 m 精确碰撞半径 |
| raycast/碰撞检查 | 线段按半体素采样，空间哈希查询 |
| 前端搜索 | 26 邻域动态 A*、局部边界、50 万体素扩展上限、路径剪枝 |
| 轨迹 | 三次 Uniform B-spline，位置/速度/加速度采样 |
| 可行性 | 根据最大速度/加速度自动拉伸时长 |
| 安全复检 | B-spline 每 0.05 s 采样并逐段碰撞检查 |

`THIRD_PARTY.md` 和 GPL LICENSE 记录请求基线和边界。当前源码没有导入上游源码或
ROS wrapper；当前核心是独立的安全占位实现。

注意：当前版本实现了比赛链路所需的 ESDF-free 体素/A*/B-spline 最小核心，
但尚未实现指定上游的 raycast 占据更新和 LBFGS 代价项。取得可达基线后应在
`flight_core` 深模块内部替换，ROS 接口和 executor 不得改变；完成该项前 EGO 票据为
未通过，不能以当前最小核心替代最终比赛验收。

## 模块关系

```text
/avoidance/lidar/pointcloud ─┐
/drone/navigation/odometry ──┼→ ego_local_planner
/drone/navigation/goal ──────┘     │
                                    ├→ /drone/navigation/trajectory
                                    └→ /drone/navigation/planned_path

/drone/navigation/trajectory ─┐
/drone/navigation/control_mode ├→ trajectory_executor @20 Hz
/drone/navigation/visual_velocity┘  └→ /fmu/in/*
```

规划器内部统一使用 Isaac 世界 `map` ENU。`px4_state_adapter` 把 NED/FRD 状态转成
ENU/FLU 并加 `px4_map_origin`；executor 发送给 PX4 前减去该原点，再做 ENU→NED。
Isaac ground truth `/drone_0_ego_odom` 只用于误差对比。

## 参数基线

参数位于 `src/drone_navigation_pkg/config/navigation.yaml`：

| 参数 | 初值 |
|---|---:|
| voxel / inflation | 0.10 m / 0.25 m |
| obstacle memory | 1.0 s |
| local horizontal / vertical | 5.5 m / 4.5 m |
| virtual ceiling | 2.9 m |
| max velocity / acceleration | 0.5 m/s / 1.0 m/s² |
| takeoff height | map z=1.8 m |
| stale hold / land | 0.3 s / 1.0 s |
| Offboard prestream | 1.0 s |

`px4_map_origin=[4.55,-0.38,1.13]` 来自 X1 当前出生点。`drop_search_pose` 仍必须从
最终比赛 USD/实景标定，默认 `[0,0,1.8]` 不能用于正式自动任务。

## 构建与验证

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs drone_navigation_pkg
source install/setup.bash
colcon test --packages-select drone_navigation_pkg
colcon test-result --verbose
```

不需要 PCL 才能构建当前核心；`PointCloud2` 由迭代器直接读取。若以后导入上游
PCL/LBFGS 实现，必须把新增依赖、commit 和许可证同步写入本文件及
`THIRD_PARTY.md`。

## 进入飞行前的硬门槛

1. 比赛场景运行，接口审计无缺失且 `/fmu/in/*` 单写入者通过。
2. direct-rotor 逐电机低速确认序号/旋向后切回 `px4`。
3. PX4 外部模拟器闭环先完成未解锁传感器票据。
4. 设置真实 `drop_search_pose`、相机朝向和视觉 PID。
5. 单独验证起飞/悬停/降落后，才允许自动地空任务门控。
