# Foxglove 只读观测桥

## 统一运行方式

Foxglove 由独立 tmux session `foxglove` 常驻，和比赛 host bringup 解耦。这样重启观测桥
不会打断导航、executor 或任务状态机。

```bash
cd /var/workspace/docker/isaac/workspace
tmux new-session -d -s foxglove \
  'bash src/bridge_competition_pkg/scripts/foxglove_daemon.sh'
```

连接 Foxglove Studio：`ws://localhost:8765`。跨机使用时优先通过 SSH 转发：

```bash
ssh -L 8765:localhost:8765 user@host
```

host bringup 不再拉起第二个 Foxglove 实例。禁止同时手工运行
`foxglove_bridge.launch.py`，否则会争用 8765。

## 控制命令（推荐用 console_scripts）

```bash
# 开机 / 重启 host 后启动
ros2 run bridge_competition_pkg foxglove_daemon start

# 改完 YAML 后一行重启（Ctrl+C，daemon 自动拉新）
ros2 run bridge_competition_pkg foxglove_daemon restart

# 看 tmux / 8765 / 日志末尾
ros2 run bridge_competition_pkg foxglove_daemon status

# 进 tmux 实时看（Ctrl+B d 退出）
ros2 run bridge_competition_pkg foxglove_daemon attach

# 完全停掉
ros2 run bridge_competition_pkg foxglove_daemon stop
```

裸 tmux 命令也兼容：

```bash
tmux ls | grep foxglove
ss -tlnp | grep 8765
tmux send-keys -t foxglove C-c  # daemon 在 2 秒后拉起新实例
```

## 安全边界

[`foxglove_bridge.yaml`](../../src/bridge_competition_pkg/config/foxglove_bridge.yaml)
只启用 `connectionGraph` capability；没有 client publish、service 或 parameter capability，
所以 Studio 不能成为第二个 `/fmu/in/*` 写入者。飞控命令只能经过
`trajectory_executor`。

白名单覆盖：

- Isaac/Pegasus `/drone0/*`、点云和 TF；
- `/drone/navigation/*`、投放视觉、舱门与 arena 状态；
- PX4 `/fmu/in/*` 和 `/fmu/out/*`。

PX4 输出的 QoS override 使用 `BEST_EFFORT`。图像或点云带宽过高时收紧 YAML 白名单，
不要改端口或开启控制 capability。

## 验证

```bash
ss -tlnp | grep 8765
ros2 node list | grep '^/foxglove_bridge$'
ros2 node info /foxglove_bridge
```

Studio 中建议使用：

| 面板 | topic | 用途 |
|---|---|---|
| 3D | `/tf`、`/drone/navigation/planned_path` | 坐标和规划轨迹 |
| Plot | `/fmu/out/vehicle_odometry` | PX4 状态曲线 |
| Image | `/drone0/down_camera/color/image_raw` | 下视视觉 |

接口与飞行票据仍以 `/tmp/drone_interface_report.json`、rosbag 和 PX4 ULog 为准，
Foxglove 画面不替代验收证据。
