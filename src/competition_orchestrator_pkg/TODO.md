# competition_orchestrator_pkg TODO（D 负责）

## 每日目标速查（看 docs/project/progress.md，不存在则参考 tasks_full.md）

| 日期 | 当日目标 |
|---|---|
| 07-17（今天）| 包骨架 + README + TODO + 文档模板 |
| 07-18 | 状态机设计完成（Graphviz 图 + 文字描述）+ 协助 A 录视频 |
| 07-19 | cargo_door_server 雏形；air_ground_coordinator 空壳 |
| 07-20 | 协调器能调 A 的 gripper + D 的 door |
| 07-21 | 端到端串通（地面完成 → 关侧舱 → 起飞 → 投放）|
| 07-22 | 全流程一镜到底录像（多次）+ 写文档 |
| 07-23 | 视频精修；技术文档 PDF 导出；录 cuRobo 加分项 |
| 07-24 | 最终打包 + md5 + 命名规范检查 |
| 07-25 | 09:00 最终检查；22:00 提交邮件 |

## 模块 M4 + M7 + M8 任务清单

### M4.1 状态机设计（07-18）
- [ ] 画状态转换图（Mermaid / Graphviz）
- [ ] 输出 `scenes/team/docs/state_machine_diagram.svg`
- [ ] 输出 `docs/state_machine_diagram.md`（mermaid 源）

### M4.2 air_ground_coordinator（07-21）
- [ ] 监听 `/arena/ground/state`
- [ ] 触发 `/demo_close_left_door` action（D 自己的）
- [ ] 触发 `/demo_drone_takeoff` action（D 自己的）
- [ ] 触发 `/demo_drone_goto` action（D 自己的）
- [ ] 监听 `/drone/drop_command`，触发 `/demo_open_bottom_door`
- [ ] 监听投放完成，触发返航

### M7.1 cargo_door_server（07-21）
- [ ] `/demo_close_left_door` action
- [ ] `/demo_open_bottom_door` action
- [ ] 发布 `/drone/cargo/left_door_cmd` JointState
- [ ] 发布 `/drone/cargo/bottom_door_cmd` JointState
- [ ] 录视频：起飞前关侧舱（任务 2.2，5 分）

### M7.2 drone_flight_server（07-21）
- [ ] `/demo_drone_takeoff` action
- [ ] `/demo_drone_goto` action
- [ ] 发布无人机任务目标，由 EGO 算法适配层生成轨迹并转换为 PX4 Offboard 设定点
- [ ] `/drone/cmd_vel` 仅保留为项目内部手动/降级控制接口，不直接驱动 PegasusSimulator 旋翼

### M8.1 文档撰写（07-22 ~ 07-23）
- [ ] 技术文档草稿（参考 `docs/project/submission_checklist.md`）
  - 场景配置说明
  - 视觉对准算法（从 B 那里拿内容）
  - 投货状态机设计（自己的）
  - 关键技术难点（2-3 个）
- [ ] 工程资料整理（USD + ActionGraph 截图 + Python 脚本）
- [ ] 文档 PDF 导出

### M8.2 录制工具脚本（07-18 ~ 07-19）
- [ ] `scenes/team/scripts/add_watermark_and_record.sh` —— ffmpeg 录屏 + 烧水印
- [ ] `scenes/team/scripts/record_template.sh` —— 通用录制启动脚本
- [ ] `scenes/team/scripts/package_submission.sh` —— 打包脚本

### M8.3 视频录制（07-22 ~ 07-24）
- [ ] 12 段必录视频（详见 `docs/project/submission_checklist.md`）
- [ ] 加分项视频（视情况）
- [ ] 全流程一镜到底（多次，取最好的一次）

### M8.4 最终打包（07-24）
- [ ] 视频精修（剪辑无效段落）
- [ ] 文档定稿 + PDF
- [ ] 工程文件 zip
- [ ] 总压缩包 + 命名 + md5

## 等待别人交付

| 我需要 | 来源 | 何时 |
|---|---|---|
| `/arena/ground/state` topic | A | 07-18 早 |
| `/drone/drop_command` topic | B | 07-21 |
| `/demo_close_left_door` action 实现（即 D 自己的）| D 自给 | 07-19 |
| `CargoDoorCommand` / `DroneFlightCommand` action 类型 | D 自给 + 加进 grasp_demo_interfaces | 07-19 |
| 全流程能跑通 | A + B + C | 07-21 |

## 我的产出物给谁

| 我交付 | 给谁 | 何时 |
|---|---|---|
| `/arena/orchestrator/state` topic | A / B | 07-21 |
| `/demo_close_left_door` action | A（确认能调） | 07-21 |
| 状态机图（SVG + MD）| 全员 + 文档 | 07-22 |
| 录屏脚本 + 水印脚本 | A / B / C（录制用）| 07-19 |
| 打包脚本 | 全员 | 07-24 |
| 视频（最终 12+ 段）| 提交 | 07-24 |
| 技术文档 PDF | 提交 | 07-24 |

## 阻塞 / 风险

- ❓ 状态机时序不一致（R7 高风险）
- ❓ 视频录制质量不达标（R8 中）
- ❓ 提交截止前发现 bug（R9 高）
