# dual_arm_pkg TODO（A 负责）

## 每日目标速查（看 docs/project/progress.md，不存在则参考 tasks_full.md）

| 日期 | 当日目标 |
|---|---|
| 07-17（今天）| 包骨架 + README + TODO |
| 07-18 | M1 装配 X1 场景拿到 prim_path_registry.yaml |
| 07-19 | cuRobo 安装完成 + hello world |
| 07-20 | 4 个基础节点跑通：observation / demo / gripper_server / pick_place |
| 07-21 | pick_place 状态机完整执行（OBSERVE → GROUND_DONE）|
| 07-22 | cuRobo 集成到 pick_place（替代关节空间插值）+ 加分项视频 |
| 07-23 | 双臂协同精度优化 |
| 07-24 | D-1 回归测试 |
| 07-25 | 提交 |

## 模块 M5 任务清单

### M5.1 基础节点（07-20 之前）
- [ ] dual_arm_observation_node：12-DOF observation pose publisher
- [ ] dual_arm_demo_node：start→demo 关节插值演示
- [ ] dual_gripper_server_node：DualGripperCommand action server
- [ ] dual_arm_pick_place_node：状态机（call detect → grip → place）
- [ ] demo_params.yaml 扩展：支持 left_arm_6 + right_arm_6 + 2 grippers

### M5.2 cuRobo 集成（07-22 之前，加分 +4）
- [ ] install cuRobo per docs/setup/env_setup.md §6
- [ ] cuRobo IK 调用代码（输入 base_frame 3D 点 → 6 joint）
- [ ] 用 cuRobo 替换关节空间直线插值
- [ ] 录视频：cuRobo 运动规划 demo（加分项 4 分）

### M5.3 双臂协同（07-23 之前，加分 +6 仅 A 平台）
- [ ] 左/右臂同时检测、同时抓、同时抬升
- [ ] 加 collision check（不撞到对方）
- [ ] 录视频：双臂协同并行抓取（加分项 6 分）

### M5.4 调参与稳定性
- [ ] gripper force 调参（保证不漏物）
- [ ] arm velocity 调参（保证不抖）
- [ ] 录视频：精准投放（1.5 任务 10 分）

## 等待别人交付

| 我需要 | 来源 | 何时 |
|---|---|---|
| `/arm_camera/{rgb,depth}_{left,right}` topic | C | 07-18 晚 |
| `/demo_grasp/object_point_base` topic | B | 07-19 晚 |
| `/demo_detect_object` action | B | 07-19 晚 |
| 双目识别（B 平台双相机）| B | 07-20 |
| `/drone/cargo/...` cmd topic（让 A 也能控制货舱备份）| C | 07-21 |

## 我的产出物给谁

| 我交付 | 给谁 | 何时 |
|---|---|---|
| `/hand_command_left/right` topic 定义 | C（用于 OmniGraph 接线） | 07-18 早 |
| `/arena/ground/state` topic 定义 | D（用于空地调度）| 07-18 早 |
| `/demo_dual_gripper_command` action | D（用于调度触发 close/open）| 07-19 晚 |
| DualGripperCommand.action 加进 grasp_demo_interfaces | D 提需求、A 写 | 07-19 晚 |
| 完整可执行 demo + 视频 | D（用于 1.5 评分项）| 07-21 |

## 阻塞 / 风险

- ❓ cuRobo GPU 兼容性（R4 风险）
- ❓ 双臂碰撞检测能不能做（R6 风险）
- ❓ X1 joint 名从 USD 出来对不对（依赖 M1）
