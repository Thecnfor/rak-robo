# Foxglove 只读观测桥

> **当前状态**：Foxglove 由 **systemd --user** 常驻守护，开机自启，挂了自动重启。
> 端口 8765 固定，所有业务节点起来后 topic **自动出现**（空白名单）。

## 接入方式

Foxglove Studio 连接 `ws://localhost:8765`。跨机：

```bash
ssh -L 8765:localhost:8765 user@host
```

⚠️ `xrak.top` 域名没做 ICP 备案，Aliyun WAF 拦截 HTTP；用 IP 直连 `ws://8.129.26.180:8765` 或 SSH 隧道绕开。

## 日常运维（systemd 命令）

```bash
# 看状态（is-active / is-enabled / status）
systemctl --user status foxglove-bridge.service

# 看日志（最近 50 行）
journalctl --user -u foxglove-bridge.service -n 50 --no-pager
# 或者直接看持久化日志（systemd StandardOutput=append 写的）
tail -f /var/workspace/docker/isaac/workspace/log/foxglove/foxglove_bridge.log

# 改完 config/foxglove_bridge.yaml 后重启
systemctl --user restart foxglove-bridge.service

# 临时停
systemctl --user stop foxglove-bridge.service

# 永久关掉（取消开机自启）
systemctl --user disable foxglove-bridge.service
```

开机自启已生效（linger=yes + WantedBy=default.target）。重启 host 之后 Foxglove **会自己起来**，不用手动操作。

## 改 YAML 之后怎么生效

`foxglove_bridge.yaml` 是 launch 时一次性加载。要让改动生效，**一行命令**：

```bash
systemctl --user restart foxglove-bridge.service
# 等 3 秒，Foxglove Studio 自动重连（WebSocket 是长连接，重启会有 1-2 秒断）
```

## 业务节点怎么接进来

**什么都不用做**。Foxglove 是空白名单，**任何 ROS 2 节点启动后，topic 自动出现在 Studio 里**：

```bash
# 起某个包
ros2 launch drone_navigation_pkg navigation.launch.py
# 起 PX4
cd /var/workspace/docker/isaac/docker/px4 && docker compose up -d
# 起 Isaac Sim 的某个 scene
# ... 所有这些都会让 Foxglove 立刻看到新的 topic，不用改任何配置
```

关掉某个节点，对应 topic 也自动消失。

## 安全边界

**写能力锁死**：YAML 里 `capabilities` 只开 `connectionGraph`，所以 Foxglove Studio
**不能** publish topic、**不能**调 service、**不能**改参数。飞控命令只能经过
`trajectory_executor` 这一个节点；Studio 无法成为第二个 `/fmu/in/*` 写入者。

**带宽边界**：空白名单下，所有 topic 都会被订阅。如果图像 / 点云占满带宽：

1. 在 `foxglove_bridge.yaml` 的 `topic_blacklist` 里加正则
2. `systemctl --user restart foxglove-bridge.service`

或者比赛录制时（提交 D-1）临时把 `topic_whitelist` 加回去（YAML 里已留好注释模板）。

## 故障排查

```bash
# 1. service 在不在
systemctl --user is-active foxglove-bridge.service

# 2. 端口在不在
ss -tlnp | grep 8765

# 3. ROS 2 节点列表（应该看到 /foxglove_bridge）
source /opt/ros/jazzy/setup.bash
source /var/workspace/docker/isaac/workspace/install/setup.bash
export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node list

# 4. ROS_DOMAIN_ID 必须一致
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"   # 应该是 45

# 5. systemd 自己的日志
journalctl --user -u foxglove-bridge.service --since "10 minutes ago"
```

## 内部命令兼容性（已废弃）

旧的 tmux 守护和 `ros2 run foxglove_daemon` 命令**已废弃**，但脚本还在仓库里以备手动 fallback：

```bash
# 手动 fallback（systemd 出问题时用）
tmux new-session -d -s foxglove \
    'bash /var/workspace/docker/isaac/workspace/src/bridge_competition_pkg/scripts/foxglove_daemon.sh'
```

如果走 fallback，**必须先停 systemd**，否则两个会抢 8765：

```bash
systemctl --user stop foxglove-bridge.service
```

## Studio 中建议面板

| 面板 | topic | 用途 |
|---|---|---|
| 3D | `/tf`、`/drone/navigation/planned_path` | 坐标和规划轨迹 |
| Plot | `/fmu/out/vehicle_odometry` | PX4 状态曲线 |
| Image | `/drone0/down_camera/color/image_raw` | 下视视觉 |
| Diagnostics | `/drone/navigation/interface_audit` | 接口票据（`/drone_interface_audit` 节点发的 String）|
| Log | (auto) | rosout 错误和警告 |

接口与飞行票据仍以 `/tmp/drone_interface_report.json`、rosbag 和 PX4 ULog 为准，
Foxglove 画面不替代验收证据。

## 相关文件

| 文件 | 作用 |
|---|---|
| `~/.config/systemd/user/foxglove-bridge.service` | systemd unit 文件 |
| `src/bridge_competition_pkg/config/foxglove_bridge.yaml` | 桥参数（白名单 / capability / QoS）|
| `src/bridge_competition_pkg/launch/foxglove_bridge.launch.py` | 桥节点 launch（端口 8765 / 0.0.0.0 写死）|
| `src/bridge_competition_pkg/scripts/foxglove_daemon.sh` | tmux fallback（已废弃，保留）|
| `log/foxglove/foxglove_bridge.log` | 桥日志（systemd 持续 append）|