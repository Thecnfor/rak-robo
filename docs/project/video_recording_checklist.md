# 视频录屏清单 (D-2.1)

> 命名严格按 `docs/project/submission_checklist.md`。
> 时长 ≤ 3 分钟 / 段；11 必交 + 5 加分 = 16 段。
> 赛段二现场顺序与安全门以 `docs/project/stage2_demo_runbook.md` 为准；本页只列镜头内容。
> 录制环境：VNC `:5900` 密码 `robo2026`，Isaac Sim + Pegasus 跑 PX4+UAV+双臂视觉。
> 录制工具：`ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i :99 ...`
> 2026-07-31 状态：Pegasus+PX4 已真实跑通完整 11 相位，禁止继续使用下文历史 SIH/
> “只到 ARMING”说法。历史统计为 2 次严格完整成功、
> 当前连续成功 3 次；结合此前样本，11 次真实投放中 10 次命中（90.9%）。
> 2026-08-01 状态：RTX LiDAR TF 根因已修复，新粗目标实飞进入
> `TARGET_SEARCH/VISUAL_ALIGN`；门禁包不计数，下一轮正式录屏并开始新的连续回归。
> 首次录屏尝试因近场返架 2 cm 高度切换抖振改为诊断票据；已实现单向下降锁存。
> 正式录屏改用 T4 `h264_nvenc`，避免 `libx264` 与 Isaac 争用 CPU；复验通过前不出片。
> 锁存复验包 `stage2_fullflow_20260801_0446` 已完整通过，并保存 1920×1080/30 fps
> 连续原片 `20260801_045136-预选赛加分1-全流程_raw.mp4`。2026-08-02 已从该单一
> 原片生成 12×等速压缩、无剪切/无拼接的正式文件
> `videos/预选赛加分1-全流程.mp4`；168.5 s、1920×1080/30 fps、H.264，满足
> ≤3 分钟要求，左上角常驻“12×加速｜全程连续无剪辑”。SHA-256：
> `751abda75fbfe6919a305b964647ef2d4f2568aab71cc9fdcb39074324789e24`。
> 第二轮 `stage2_fullflow_20260801_0537` 也已严格通过，投放误差 0.915 cm；该轮未
> 额外录屏，以避免编码负载干扰连续回归。
> 第三轮 `stage2_fullflow_20260801_0618` 严格通过，投放误差 1.090 cm；连续计数
> 更新为 3/10。

## 命名规则（不能动）
```
预选赛赛段{1,2}任务{1..6}-{name}.mp4
预选赛加分{1..5}-{name}.mp4
```
XX队占位 → 全局替换为真实队名。

---

## 必交 10 段

### 段 1 — 预选赛赛段1任务1-仿真基础配置
**内容**：
1. VNC 打开 Kit，看 `X1_race_scene.usd` 装配完整
2. `ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py` 拉起 9+ 节点
3. `ros2 node list | sort` 截图（注意去重）
4. Foxglove Studio 连 `ws://localhost:8765`，展示 9 个 panel：
   - 3D：/tf + /drone/navigation/planned_path
   - Plot：/fmu/out/vehicle_odometry
   - Image：/drone0/down_camera/color/image_raw
   - Diagnostics：/drone/navigation/interface_audit
5. `ros2 run bridge_competition_pkg chain_status` 输出

**时长**：3 分钟
**重点**：证明场景 + host chain + Foxglove 都正常

### 段 2 — 预选赛赛段1任务2-自主导航
**内容**：
1. 启动 `nav2_navigation.launch.py`（地图已加载）
2. `python3 src/nav2_demo_pkg/scripts/send_goal.py --x 2.0 --y 3.0 --yaw 1.57`
3. Foxglove 看 X1 朝目标走，避开障碍
4. 终点到达，任务完成

**时长**：2 分钟

### 段 3 — 预选赛赛段1任务3-识别位姿
**内容**：
1. 启动 `perception_pipeline_demo.launch.py`
2. Foxglove 看 `/arm_camera/rgb` 实时图 + `/demo_grasp/{bbox, label, confidence}`
3. 移动 pencil 入画 → bbox 出现，label="pencil"
4. 移到画面中心 → `object_point_base` 更新
5. 注：当前 yoloe 是 stub 模式，bbox 始终在中心；可手动改 `simulate_target_label` 演示多种物体

**时长**：2 分钟

### 段 4 — 预选赛赛段1任务4-抓取
**内容**：
1. `ros2 launch grasp_demo_pkg perception_pipeline_demo.launch.py`
2. `ros2 action send_goal /demo_gripper_command grasp_demo_interfaces/action/GripperCommand '{command: open}'`
3. Foxglove 看 `/hand_command` JointState 的 `gripper_*_joint` 跳到 0.04
4. 同一 goal `close`，跳回 0.0
5. `dual_gripper_server_node` 同步模式：左右夹爪联动

**时长**：2 分钟

### 段 5 — 预选赛赛段1任务5-投放
**内容**：
1. 注：无人机投放是 D-3.4 砍掉的部分
2. 用 SIH 模式做 1 次 cargo_status_sim 演示：
   - 启动 `cargo_status_sim` + `ground_state_sim` + `mission_trigger`
   - Foxglove 看 `/cargo_bay/command` 在 left_close / bottom_open 切换
   - `/arena/ground/state` 出现 COMPLETE
3. 演示一次投放状态机：`OBSERVE → DETECTING → GRASPING → LIFTING → PLACING → COMPLETE`

**时长**：2 分钟

### 段 6 — 预选赛赛段2任务1-仿真基础
**内容**：
1. 启动 `isaacsim51-scene.service` 之前状态（无）
2. 启动后：`ros2 topic list | grep /drone0/ | head` + `grep /fmu/`
3. 切换 PX4 后端：`PX4_SIM_MODEL=sihsim_quadx` 演示双模式
4. `cat /var/workspace/docker/isaac/logs/isaacsim51-scene.log | tail -20` 演示 scene ready

**时长**：2 分钟

### 段 7 — 预选赛赛段2任务2-关侧舱
**内容**：
1. 进入 VNC，确认门当前是 `left_opened`（默认）
2. 在 Host 终端执行 `bash docs/project/stage2_demo_control.sh close-doors`（已设 `CONFIRM_SCENE_COMMAND=YES`）
3. VNC 中看到侧舱门关闭，VNC 右侧 `/cargo_bay/status` 出现 `left_closed bottom_closed payload_locked=True prearm_support=True`
4. 文字说明：必须在 PX4 启动前完成，否则 EKF 会把关门瞬态当作姿态原点

**时长**：2 分钟

### 段 8 — 预选赛赛段2任务3-起飞飞行
**内容**：
1. 启动 PX4 Docker，确认 `/fmu/out/vehicle_status_v1` 实时
2. 启动 Host bringup（`stage2_demo_control.sh host`），rosbag 自动带时间戳
3. `chain_status` 与 `drone_interface_audit` 报告 `ok=true`
4. 比赛默认保留 `mission_autostart=false`；触发方式为 `CONFIRM_FLIGHT=YES bash docs/project/stage2_demo_control.sh mission`，说明只发一次地面 COMPLETE，orchestrator 内部一次性派发，**不叠加 `mission_trigger`**
5. Foxglove 看 `/drone/navigation/state` 走 `IDLE → PREFLIGHT → ARMING → TAKEOFF`；如未到 TAKEOFF 则说明本段只到 ARMING 阶段

**时长**：2 分钟

### 段 9 — 预选赛赛段2任务4-视觉对准
**内容**：
1. `drone_target_detector_node` 启动（已含在 host chain）
2. VNC 里把红色 marker 放在 down_camera 视野下
3. Foxglove 看 `/drone/drop_target_offset` 4 个 float：`[nx, ny, area_fraction, radius_px]`
4. 移到中心 → `area_fraction` 上升，`(nx, ny) → 0`
5. `/drone/drop_command` 跳到 true

**时长**：2 分钟

### 段 10 — 预选赛赛段2任务5-投放执行
**内容**：
1. 在完整 Pegasus+PX4 任务中连续录制 `VISUAL_ALIGN → DROP_HOLD`，不得切镜头或手工开门
2. VNC 中看到真实底舱门打开、payload 落入圆内，状态为 `bottom_opened payload_released`
3. 继续保留 `RETURN → LAND → COMPLETE` 与自动 disarm 画面
4. 隔离的 `release-payload` 命令只用于机构烟测，不得代替本段飞行证据

**时长**：2 分钟

### 段 11 — 预选赛赛段2任务6-精准度
**内容**：
1. 有真实投放才量落点；没有则本段只展示视觉误差、Foxglove 落点叠加和说明
2. 调整 `navigation.yaml` 里的 `visual_kp=0.25` 前后对比（不修改默认值），输出 `/drone/navigation/visual_velocity`
3. 文字说明：比赛验收需 90% 落点 ≤ 0.2 m，无真实投放不得伪造距离

**时长**：2 分钟

---

## 加分 5 段

### 加分 1 — 全流程
**内容**：
1. 同一 VNC 连续录制真实 X1 场景、Pegasus、PX4 和 host chain；不得使用 SIH、
   `cargo_status_sim`、重复触发或强制切模式
2. 展示一次真实地面 `COMPLETE` 后完整经过 `IDLE → PREFLIGHT → ARMING → TAKEOFF →
   EGO_TRANSIT → TARGET_SEARCH → VISUAL_ALIGN → DROP_HOLD → RETURN → LAND → COMPLETE`
3. 同屏保留门动作、下视画面、规划轨迹、落点和最终 disarm；原始长录像不得拼接

**时长**：3 分钟

### 加分 2 — 双目识别
**内容**：
1. yoloe stub 同时订阅 `/arm_camera/rgb_left` + `/arm_camera/rgb_right`（需要单独接 Isaac camera prim 才有 2 路）
2. 当前实现是单路；演示 stub 模式，文字说明："代码已留双目接口，待真实 YOLOE weights 部署后激活"
3. 画外音解释双目的视差原理和 3D 距离计算

**时长**：2 分钟

### 加分 3 — 双臂协同
**内容**：
1. `dual_gripper_server_node` 的左/右夹爪同步打开 + 同步关闭
2. 演示同时发 2 个 goal（left 独立，right 独立）
3. `/hand_command` JointState 的 2 个 gripper_joint 同步变化
4. 文字说明：`dual_arm_pkg/dual_arm_pick_place_node` 状态机已实现 OBSERVE→COMPLETE，碰撞检测留给 M5.3

**时长**：2 分钟

### 加分 4 — cuRobo
**内容**：
1. **注**：cuRobo 集成是 M5.2（待办）
2. 本段演示当前关节空间直线插值：`plan_to_pose_node` 生成 5 个候选位姿（pre-grasp, grasp, lift, pre-place, place）
3. Foxglove 看 `/hand_command` 的 12-DOF 序列
4. 文字说明："计划用 cuRobo IK 替换关节空间直线插值，文档见 docs/project/技术文档-XX队-预选赛.md §5 难点 4"

**时长**：2 分钟

### 加分 5 — 无人机投放突出
**内容**：
1. 演示真实完整 11 相位及 EGO、下视视觉 PID、底舱释放和精准落点
2. 引用严格 rosbag 审计的逐轮落点数据；未达到至少 10 次/90% 前必须明确标注“统计未关闭”
3. 展示 `/fmu/in/*` 唯一写入者与无 PX4 failsafe 证据，不再引用历史 SIH Offboard quirk

**时长**：2 分钟

---

## 录制步骤

```bash
# 1. 启动 Isaac + chain
sudo systemctl start isaacsim51-scene.service
cd /var/workspace/docker/isaac/docker/px4
docker compose up -d   # 默认 gazebo-classic_iris，或 PX4_SIM_MODEL=sihsim_quadx 上拉
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py

# 2. 另一终端
vncviewer :99    # 打开 Isaac Sim 视窗
# 录屏
ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i :99 \
    -c:v libx264 -preset fast -crf 23 -pix_fmt yuv420p \
    videos/预选赛赛段1任务1-仿真基础配置.mp4

# 3. 录完按 q。ctrl+c 结束 ffmpeg
```

## 总时长预算
- 必交 10 段 × 2-3 分钟 = ~25 分钟
- 加分 5 段 × 2-3 分钟 = ~12 分钟
- 录制总耗时 ≈ 90 分钟（包含 setup + 调试）
