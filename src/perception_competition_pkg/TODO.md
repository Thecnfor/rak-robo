# perception_competition_pkg TODO（B 负责）

## 每日目标速查（看 docs/project/progress.md，不存在则参考 tasks_full.md）

| 日期 | 当日目标 |
|---|---|
| 07-17（今天）| 包骨架 + README + TODO |
| 07-18 | yoloe_detector_node 跑通现有 grasp_demo_pkg 的代码 |
| 07-19 | DetectObject action server 跑通；depth_pose_estimator_node 跑通；TF 链校好 |
| 07-20 | 双目相机（A 平台加分项）：左/右同时检测，融合 |
| 07-21 | 无人机下视相机视觉对准：圆心检测 + 归一化偏移输出 |
| 07-22 | 视觉对准稳定性优化（光照变化、加权滤波）|
| 07-23 | D-1 回归测试 + 录视频（任务 3 物料识别）|
| 07-24 | 视频精修 + 文档 |
| 07-25 | 提交 |

## 模块 M6 + 感知部分 M5 任务清单

### M6.1 地面物料检测（07-20 之前）
- [ ] yoloe_detector_node：复用 grasp_demo_pkg 的实现，改成 service / action
  - 输入：`/arm_camera/rgb_left` 和 `/arm_camera/rgb_right`
  - 输出：bbox + mask + 类别 + 置信度
  - 提供 action：`/demo_detect_object`
- [ ] dual_yolo_fusion_node：双目融合（可选，A 平台加分项 5 分）
- [ ] 录视频：物料识别与位姿估计（任务 1.3，10 分）

### M6.2 位姿估计（07-21 之前）
- [ ] depth_pose_estimator_node：深度 median + mask PCA → 相机系 3D 点 + 长轴
- [ ] tf_to_base_node：相机系 → base_link_arm 系
- [ ] 输出：`/demo_grasp/object_point_base` 等 topic
- [ ] 录视频：位姿估计（任务 1.3，10 分）

### M6.3 无人机视觉对准（07-22 之前）
- [ ] drone_ground_target_detector：HSV 阈值 + 形态学 + minEnclosingCircle
  - 输入：`/drone/down_camera/rgb`
  - 输出：`/drone/drop_target_offset` `[nx, ny, area, radius]`
- [ ] drone_alignment_error_filter：滑动窗口平滑（抑制控制抖动）
- [ ] 录视频：机载视觉对准（任务 2.4，8 分）

### M6.4 投放决策（07-21）
- [ ] drone_drop_decider：读 depth → 高度判断 + alignment error < 阈值 → 发 `/drone/drop_command` (Bool)
  - 注意：这个节点是否在 B 包里有争议 —— D 的状态机也读它
  - **决定**：B 只发 drop_command 信号，D 监听

### M6.5 自适应调参（07-22，加分项）
- [ ] HSV 范围自适应（光线变化）
- [ ] B 算法介入调参窗口
- [ ] 录视频：无人机投放表现突出（加分项 5 分）

## 等待别人交付

| 我需要 | 来源 | 何时 |
|---|---|---|
| `/arm_camera/{rgb,depth}_{left,right}` topic | C | 07-18 晚 |
| `/drone/down_camera/{rgb,depth}` topic | C | 07-18 晚 |
| TF：`map → odom → base_link → base_link_arm → arm_Camera_left/right` | C（Isaac PublishTF）| 07-18 晚 |
| TF：`map → avoidance_base_link → down_camera` | C | 07-21 |
| 物料 3D 模型（用于 cuRobo 抓取参考）| A | 07-21 |

## 我的产出物给谁

| 我交付 | 给谁 | 何时 |
|---|---|---|
| `/demo_grasp/object_point_base` topic 定义 | A | 07-18 早 |
| `/drone/drop_target_offset` topic 定义 | D | 07-18 早 |
| `/demo_detect_object` action | A | 07-19 晚 |
| `/drone/drop_command` topic 定义 | D | 07-21 |

## 阻塞 / 风险

- ❓ 双目相机有没有（依赖 M1 A 装配）
- ❓ YOLOE 在 Isaac 渲染图上准不准（可能要二次训练）
- ❓ 圆心检测在不同光照下稳不稳定（R2 R5）
- ❓ AprilTag 不能用 → 必须用色块或几何形状
