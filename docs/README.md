# Docs · 文档索引

> 第二十五届 ROBOTAC AIROBOTIC 预选赛工作区文档。
> 顶层入口：从这里按角色/任务找到需要的文档。

## 目录结构

```
docs/
├── README.md                       ← 你正在看
├── project/                        ← 项目元信息（任务/提交）
│   ├── tasks_full.md               ← M1~M9 模块任务清单
│   └── submission_checklist.md     ← 提交清单（视频/文档/打包）
├── setup/                          ← 环境 / 安装 / 集成方案
│   ├── env_setup.md                ← 总环境方案（PX4 SITL + PegasusSimulator + Isaac Sim 5.1）
│   ├── ego_planner_integration.md  ← EGO-Planner 复用算法库 + 项目适配层设计
│   └── foxglove_setup.md           ← Foxglove WebSocket 桥（常驻观测工具）
├── contracts/                      ← 包间契约（最常被改的）
│   └── interface_contracts.md      ← topic / action / frame / param 契约
├── runbooks/                       ← 任务操作手册 + 模板
│   ├── M1_modeling_runbook.md      ← M1 Isaac Sim 装配 8 步走
│   ├── M2_omni_bridge_spec.yaml    ← M2 OmniGraph ↔ ROS2 桥配置模板
│   ├── M2_bridge_smoke_test.py     ← 接口审计兼容入口
│   └── drone_navigation.md         ← 无人机全链路启动/标定/验收
└── competition/                    ← 比赛官方原文（不改）
    ├── 第二十五届全国大学生机器人大赛ROBOTAC AIROBOTIC赛项 预选赛评审细则-0626.docx
    └── 预选赛作品提交规则(1).docx
```

## 阅读路径

### 🆕 新人加入（5 分钟）
1. [`contracts/interface_contracts.md`](contracts/interface_contracts.md) — 知道 4 个包之间用哪些 topic/action 沟通
2. [`project/tasks_full.md`](project/tasks_full.md) — 知道整体进度
3. [`CLAUDE.md`](../CLAUDE.md) — 工作区总规约（环境、构建、运行）

### 🔧 调环境 / 重装
1. [`setup/env_setup.md`](setup/env_setup.md) — 装 PegasusSimulator / Isaac Sim 5.1 / cuRobo / ROS 2 Jazzy
2. [`../src/px4_sitl_usage.md`](../src/px4_sitl_usage.md) — PX4 SITL Docker 启动 / DDS 桥接验证
3. [`setup/foxglove_setup.md`](setup/foxglove_setup.md) — 装 foxglove_bridge 看 ROS 2 topic
4. [`setup/ego_planner_integration.md`](setup/ego_planner_integration.md) — EGO 算法库接入

### ✏️ 改 topic / action / frame
> ⚠️ code-as-spec 真值源在 `bridge_competition_pkg/interface_audit.py` 的 `DEFAULT_REQUIRED_TOPICS`
1. 改 [`contracts/interface_contracts.md`](contracts/interface_contracts.md)
2. 改 `interface_audit.py` 的 `DEFAULT_REQUIRED_TOPICS`（必须）
3. 团队群 ack
4. 同 PR 改代码

### 🚁 调无人机 / PX4
1. [`runbooks/drone_navigation.md`](runbooks/drone_navigation.md) — 全链启动、接口票据、标定和放飞顺序
2. [`../src/px4_sitl_usage.md`](../src/px4_sitl_usage.md) — PX4 Docker / DDS 诊断
3. [`setup/ego_planner_integration.md`](setup/ego_planner_integration.md) — 规划边界与参数

### 🛰️ 调试 / 看 topic
> Foxglove 已经常驻在 8765，不用单独启
1. [`setup/foxglove_setup.md`](setup/foxglove_setup.md) — 连接方式 + 布局模板 + 端口冲突排查

### 🎬 提交前
1. [`project/submission_checklist.md`](project/submission_checklist.md) — 视频/文档/打包/截止时间
2. [`project/tasks_full.md`](project/tasks_full.md) — M9 集成测试全绿
3. [`competition/`](competition/) — 评分细则 / 提交规则（必读）

## 与源码的对应

| 文档位置 | 源码位置 |
|---|---|
| `setup/ego_planner_integration.md` | `src/drone_navigation_pkg/`（flight_core / ego_local_planner / trajectory_executor） |
| `setup/foxglove_setup.md` | `~/.config/systemd/user/foxglove-bridge.service`（systemd 守护）+ `src/bridge_competition_pkg/launch/foxglove_bridge.launch.py` + `config/foxglove_bridge.yaml`（与 host_bridge_bringup 解耦） |
| `setup/env_setup.md` § PegasusSimulator | `src/bridge_competition_pkg/` 的 scene_app_x1_5_1_native.py |
| `runbooks/M1_modeling_runbook.md` | `scenes/team/environments/<team>_office_X1_full.usd`（A 在 Kit VNC 里做） |
| `runbooks/M2_omni_bridge_spec.yaml` | Isaac ActionGraph 节点配置（在 Kit Script Editor 里做）|
| `runbooks/M2_bridge_smoke_test.py` | `src/bridge_competition_pkg/scripts/` 复制成可执行脚本 |
| `contracts/interface_contracts.md` | 全部 4 个 src/<pkg>/ 的接口都按它来 |

## 维护规则

- **改 topic / action 前**：跟团队群 ack，code-as-spec 是 `interface_audit.py` 的 `DEFAULT_REQUIRED_TOPICS`
- **改文档后**：检查同 PR 里其它文档的交叉引用是否需要更新
- **新增文档**：放进合适的子目录，更新本文档的目录结构
- **删除文档**：先 grep 引用，再删；本文档的目录结构也要同步
- **不重命名文件**：保留文件名 / git 历史，分层只靠目录
