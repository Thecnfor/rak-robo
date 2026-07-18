# drone_navigation_pkg

比赛无人机唯一导航/控制包（GPL-3.0）。它把 PX4 状态适配、EGO 风格局部规划、
20 Hz Offboard 执行和全程任务状态机收在一个有明确边界的包中。

## 组件

- `px4_state_adapter`：PX4 NED/FRD → ROS `map` ENU/FLU。
- `ego_local_planner`：点云体素地图、动态 A*、Uniform B-spline 与碰撞复检。
- `trajectory_executor`：唯一 `/fmu/in/*` 写入者，先预流、后 Offboard、再解锁。
- `flight_supervisor`：舱门、起飞、规划、视觉投放、返航、降落和安全状态机。

## 安全默认值

- `mission_autostart=false`；地面状态成功后由 orchestrator 发送任务请求。
- 无新鲜 PX4 odometry 和点云时禁止进入 Offboard。
- 数据超过 0.3 s 悬停，超过 1.0 s 请求 Land；PX4 failsafe 立即请求 Land。
- `drop_search_pose` 默认值只是占位，正式飞行前必须标定。

## 构建/测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs drone_navigation_pkg
source install/setup.bash
colcon test --packages-select drone_navigation_pkg
```

完整接口和启动顺序见
[`docs/contracts/interface_contracts.md`](../../docs/contracts/interface_contracts.md) 与
[`docs/runbooks/drone_navigation.md`](../../docs/runbooks/drone_navigation.md)。
