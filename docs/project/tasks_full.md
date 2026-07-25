# 任务清单（按模块）

## M1 机器人建模（A · VNC Kit GUI）
- [ ] 加载 `active/worlds/X1_race/X1_race_scene.usd`
- [ ] 装 Mercury X1 + Sunray + air_fpv_box
- [ ] 装 X1 双臂相机 + Sunray 下视相机 + X1 LiDAR
- [ ] 物料摆放（pencil 等）
- [ ] 保存 `active/worlds/<team>_office_X1_full.usd`
- [ ] 整理 prim_path_registry.yaml

## M2 OmniGraph ROS2 桥（C）
- [x] PegasusSimulator 集成脚本：PX4/observer 与 direct-rotor 互斥模式、点云、里程计、下视相机、舱门
- [x] **PX4 v1.16.2 Docker 外部模拟器模式**
- [x] `drone_navigation_pkg` 滚动体素/A*/B-spline 规划核心、PX4 状态适配和 20 Hz 唯一 executor（EGO 上游 commit 不可达，2026-07-20 用户裁决用本地最小核心**替掉**，见 `docs/setup/ego_planner_integration.md` 与 `docs/project/m2_evidence.md`）
- [x] host 端总 bringup、视觉、地空协调和接口审计
- [x] 真实比赛场景接口票据：必备 topic/QoS/频率/电机映射全部通过（2026-07-20 `drone_interface_audit ok=true` + `chain_status ok=true`，见 `docs/project/m2_evidence.md`）
- [ ] Isaac 端加 ROS2 节点：SubscribeTwist/JointState（PublishTF/Clock/Image/LaserScan 已在线）
## M3 ROS2 host bringup（C）
- [x] 编译导航、视觉、协调、bridge、px4_msgs、grasp_demo_interfaces
- [x] nav2 + dual_arm + perception + bridge 全 launch

## M4 空地协同调度（D）
- [x] air_ground_coordinator 状态机
- [x] 状态机图（Mermaid/Graphviz）

## M5 双臂驱动（A）
- [x] dual_arm_observation / demo / gripper_server / pick_place
- [ ] cuRobo IK 集成（加分 +4；占位 plan_to_pose 服务已实现）
- [ ] 双臂协同并行抓取（加分 +6；pick_place 状态机已实现）
- [x] DualGripperCommand action 加进 grasp_demo_interfaces（已有 GripperCommand 复用）
## M6 视觉对准（B）
- [x] yoloe_detector_node（DetectObject action）
- [x] depth_pose_estimator_node + TF
- [x] drone_target_detector（HSV + minEnclosingCircle）
- [x] supervisor 视觉稳定/速度/偏差投放门控（参数待实景标定）
## M7 货舱控制（D）
- [x] `/cargo_bay/door_command` action 与 `/cargo_bay/{command,status}` 适配
- [x] CargoDoorCommand / DroneFlightCommand actions 加进 grasp_demo_interfaces

## M8 文档 + 提交（D）
- [ ] 技术文档 PDF
- [ ] 12 段视频 + 5 段加分视频
- [ ] ffmpeg 水印 + 录屏脚本
- [ ] 打包 + md5

## M9 集成测试（A）
- [x] M9.5 /demo_detect_object → 检测出 pencil（已验证：result.success=true detected_class=pencil）
- [ ] M9.1 World.play() → host topic 全在
- [ ] M9.2 cmd_vel → X1 动
- [ ] M9.3 hand_command → 双臂 + 双夹爪动
- [ ] M9.4 PX4 Offboard 预流→解锁→起飞→悬停→Land（2026-07-22 可见无人机已多次受控起降；最终安全构建的 `round1i` 探针 `success=true`，正式 1.8 m/30 s 与重复稳定性仍待验收，见 `drone_hover_evidence_2026-07-22.json`）
- [ ] M9.6 全流程一镜到底
