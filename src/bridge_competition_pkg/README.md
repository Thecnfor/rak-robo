# bridge_competition_pkg

Isaac/Pegasus ↔ host 的 bringup 与接口诊断包。它不拥有 EGO 算法、业务状态机或
PX4 Offboard 设定点。

## 提供内容

- `host_bridge_bringup.launch.py`：一次启动导航、视觉、协调、接口审计、Foxglove，
  可选 rosbag。
- `drone_interface_audit`：记录实际 topic 类型、publisher/subscriber、QoS，并检查
  `/fmu/in/*` 单写入者。
- `direct_rotor_smoke_test`：只在 `DRONE_BACKEND=direct_rotor` 下、显式
  `enabled:=true` 时逐电机低速测试，退出时四电机归零。
- `foxglove_bridge.launch.py`：只读观测入口，固定端口 8765。

规划、PX4 状态转换、Offboard 执行和任务状态机在 `drone_navigation_pkg`；地空业务
action 在 `competition_orchestrator_pkg`。

## 构建与运行

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bridge_competition_pkg
source install/setup.bash
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
```

接口契约：
[`docs/contracts/interface_contracts.md`](../../docs/contracts/interface_contracts.md)。
全链路步骤：
[`docs/runbooks/drone_navigation.md`](../../docs/runbooks/drone_navigation.md)。

## 约束

- `px4` 与 `direct_rotor` 模式不能同时运行。
- bridge 禁止发布 `/fmu/in/offboard_control_mode`、`trajectory_setpoint`、
  `vehicle_command`。
- 文档预期 topic 不能代替实际接口报告；场景未加载时审计失败是正确结果。
