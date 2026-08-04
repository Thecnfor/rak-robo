# 本地开发 runbook（无 Isaac Sim 的代码开发）

本地机只开发代码；Isaac Sim / PX4 SITL / 真机验证永远在 Socl
（`/var/workspace/docker/isaac/workspace`）。GitHub 是唯一同步通道：
本地 commit → push → Socl `git pull`。

## 环境事实（2026-08-05 基线）

| | 本地 | Socl |
|---|---|---|
| ROS 2 | Lyrical（Ubuntu 26.04，Python 3.14） | Jazzy（Python 3.12） |
| 角色 | 代码开发 + 逻辑回归 | 权威构建 + 仿真/真机 |
| GUI | Foxglove Studio、rviz2、rqt、PlotJuggler 可用 | 无头（foxglove_bridge 常驻 8765） |

本地构建产物只是测试沙盒，不与 Socl 共享。

## 命令基线

```bash
# 注意：本机交互 shell 是 zsh，ROS setup.bash 必须在 bash 里 source
bash -c 'source /opt/ros/lyrical/setup.bash && source install/setup.bash && <cmd>'

# 构建（跳过本地无法/无需构建的包）
colcon build --packages-skip nav2_demo_pkg px4_ros_com

# 测试（77 用例全绿为基线）
colcon test --packages-select drone_navigation_pkg perception_competition_pkg \
  competition_orchestrator_pkg bridge_competition_pkg dual_arm_pkg
colcon test-result --verbose

# 快速回归（无需构建，秒级）
python3 -m pytest src/bridge_competition_pkg/test/ src/drone_navigation_pkg/test/test_hover_probe_core.py -q
```

## 本地跳过的包及原因

- `nav2_demo_pkg` —— 依赖 Nav2 栈，但 **Nav2 尚未为 Lyrical 发布到
  packages.ros.org**（新发行版窗口期；Jazzy 上有 `ros-jazzy-navigation2`）。
  该包只含 launch/config，无代码、无测试，实际导航栈在 Socl 的 Jazzy 跑。
  等 Lyrical 的 Nav2 上 apt 后再启用。
- `px4_ros_com` —— vendored 上游示例，使用 Lyrical 已移除的
  `ament_target_dependencies`；非比赛链路，不修。
- `grasp_demo_pkg` / `isaac_ros2_control` —— 无 test/ 目录，不进测试集。

## 观测链路（本地 GUI + 远程仿真）

Socl 的 `foxglove-bridge.service`（systemd --user）常驻 0.0.0.0:8765：

```bash
ssh -N -L 8765:127.0.0.1:8765 Socl &     # 隧道
# Foxglove Studio → ws://localhost:8765
```

本地开发**不设** `ROS_DOMAIN_ID=45`（那是真机域）；本地无 DDS 对端，
任何误操作都 fail-closed。

## 已知差异

- Lyrical 移除了 `ament_target_dependencies()`：新 C++ 目标一律用
  `target_link_libraries(tgt ${pkg}_TARGETS)`（Jazzy 同样支持）。
- Python 3.14 的 setuptools 弃用 `setup.py test`/`tests_require`，
  ament_python 包测试走 `test_suite='test'` + `test/__init__.py`
  （unittest 发现）或 pytest 配置；参考 bridge_competition_pkg 与
  dual_arm_pkg 的现有写法。
