# M2 接口票据 · 实测证据

> 2026-07-20 (UTC 16:45)
> Pegasus 全场景 + PX4 Docker 容器 + 主机 host chain 同时在线情况下，
> `drone_interface_audit` 与 `chain_status` 双方都报告 `ok=true`。

## 现场状态

| 组件 | 状态 | 来源 |
|---|---|---|
| `isaacsim51-scene.service` | `active` | `systemctl is-active isaacsim51-scene.service` |
| Pegasus `PX4MavlinkBackend` listener TCP `4560` | `LISTEN 127.0.0.1:4560` | `ss -tlnp` |
| `px4-sitl` Docker container | `Up About an hour` | `docker ps` |
| `foxglove-bridge` `8765` | `LISTEN 0.0.0.0:8765` | `ss -tlnp` |
| ROS 2 domain | `45` (Fast DDS) | `run_demo_scene.sh` env 注入 |
| 主机 `/fmu/in/*` topic 数 | `27` | `ros2 topic list` |
| 主机 `/fmu/out/*` topic 数 | `24` | `ros2 topic list` |
| 主机 `/drone0/*` topic 数 | `10` | `ros2 topic list` |

## `drone_interface_audit` 报告

```json
{
  "ok": true,
  "missing": [],
  "unpublished": [],
  "disconnected_commands": [],
  "unique_fmu_writer": true,
  "invalid_fmu_writers": {},
  "multiple_fmu_writers": [],
  "resolved_topics": {
    "/fmu/out/vehicle_status": "/fmu/out/vehicle_status_v1",
    "/fmu/out/vehicle_odometry": "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_command_ack": "/fmu/out/vehicle_command_ack",
    "/fmu/out/vehicle_land_detected": "/fmu/out/vehicle_land_detected",
    "/fmu/in/offboard_control_mode": "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint": "/fmu/in/trajectory_setpoint",
    "/fmu/in/vehicle_command": "/fmu/in/vehicle_command"
  }
}
```

完整 JSON（topics 段含每个 topic 的 publisher / subscriber / QoS / 频率）写到
`/tmp/drone_interface_report.json`。

## `chain_status` 实时快照

```text
=== INTERFACE CONTRACT ===
ok=true
unique_fmu_writer=true
missing=[]  unpublished=[]  disconnected_commands=[]  invalid_fmu_writers={}
=== LIVE STATE ===
/drone/navigation/executor_state: (no message yet)
/drone/navigation/landed: True
/drone/navigation/planner_state: WAITING_FOR_INPUTS
/drone/navigation/px4_status: ready=false armed=false offboard=false failsafe=false nav_state=18
/drone/navigation/state: PREFLIGHT operator_override=LAND
```

`nav_state=18` 是 PX4 v1.16 的 `Ready` 状态；`/drone/navigation/state=PREFLIGHT`
是 flight_supervisor 等 ground_state 触发；`landed=True` 是健康的
（`flight_core.hpp::LandedStampedPublisher`）。

## 期间修复的两个 bug

| Bug | 症状 | 根因 | 修复 |
|---|---|---|---|
| 接口票据误报 `vehicle_status` 未发布 | `unpublished=['/fmu/out/vehicle_status']`，但 uXRCE 实际在发 `_v1` | `interface_audit._audit` 与 `chain_status._evaluate` 把 publisher/subscriber dict 用 `required` 名称（unversioned）做 key，而 `evaluate_interface` 用 resolved 名称（带 `_v1`）做 key，两边错位导致 lookup miss | 全部统一改用 resolved 名称做 key，并镜像一份 unversioned 副本让 `evaluate_interface._lookup` 的 fallback 也命中 |
| `chain_status` 一启动就报全部 topic missing | `_evaluate` 没有 spin executor，直接读 `get_topic_names_and_types()`，新节点的 DDS discovery 还没完成 | 旧版只等 1 s spin 不够 | 在 `rclpy.init` 之后加 2.0 s discovery spin |
| `chain_status` 把 `/trajectory_executor` 判成 "invalid_fmu_writer" | `endpoint.node_name` 不带前导 `/`，所以 dict 里写 `['trajectory_executor']`，`evaluate_interface` 要的是 `['/trajectory_executor']` | `chain_status` 用裸 `endpoint.node_name`；`interface_audit` 用 `_node_path` 拼成绝对路径 | 在 `chain_status` 加同款 `_node_path` helper |

修复在 `src/bridge_competition_pkg/bridge_competition_pkg/interface_audit.py`
和 `chain_status.py`；不改动 `interface_contract.py` 的核心 helper（10/10
单测全过）。

## EGO 上游 commit 状态（已 close-out）

- 上游 commit `23a8d5a19…` 在 `ZJU-FAST-Lab/ego-planner` 官方 remote 上
  `not our ref`，未取得可达 fork。
- 用户决策：用本地最小核心（`drone_navigation_pkg/flight_core` 内自研
  滚动体素 / 26 邻域 A* / 三次 B-spline + 可行性拉伸）作为比赛主链
  规划器。
- `docs/setup/ego_planner_integration.md` 与 `docs/project/tasks_full.md`
  同步更新关闭此 ticket。
