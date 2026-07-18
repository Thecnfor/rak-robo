# bridge_competition_pkg 后续票据

规划和 PX4 Offboard 已迁移到 `drone_navigation_pkg`；本包只保留传输、bringup、接口
审计和 direct-rotor 烟测。

## 已实现

- [x] `host_bridge_bringup.launch.py` 启动导航、视觉、协调、审计、Foxglove、可选 bag
- [x] `drone_interface_audit` 记录类型/端点/QoS并检查 `/fmu/in/*` 单写入者
- [x] `direct_rotor_smoke_test` 显式解锁、限速、逐电机、finally 归零
- [x] PX4/Isaac 与 direct-rotor 两种接口清单

## 必须在真实场景完成

- [ ] 保存 `px4` 模式 `/tmp/drone_interface_report.json`，无缺失
- [ ] direct 模式冻结 rotor0..3 的物理位置和旋向，随后切回 `px4`
- [ ] 测量 raw topic、点云、相机、PX4 topic 实际频率和时间戳
- [ ] 证明 `/fmu/in/*` 每项只有 `trajectory_executor` 一个 publisher
- [ ] 完成 30 秒悬停、失联 Land、障碍重规划和完整任务 rosbag

详见 [`docs/runbooks/drone_navigation.md`](../../docs/runbooks/drone_navigation.md)。
