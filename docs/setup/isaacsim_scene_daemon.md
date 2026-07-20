# Isaac Sim 5.1 — Cargo Delivery 场景守护

> 2026-07-18 落地。把空 Kit daemon 替换成"挂上 PX4 / uXRCE / ROS bridge 全链"的 scene
> daemon。空 Kit daemon 保留为 `isaacsim51.service`(默认 disable),只用于手画
> ActionGraph / OmniGraph 调试。

## 拓扑

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ HOST                                                                        │
│                                                                             │
│  xvfb-isaac.service   Xvfb :99 -screen 0 1920x1080x24 -ac +extension GLX    │
│  x11vnc-isaac.service x11vnc -display :99 -rfbport 5900                     │
│                          (auth /home/socl/.vnc/passwd = robo2026)            │
│                                                                             │
│  isaacsim51.service    空 Kit daemon (default DISABLED; 保留作 M2 手画)      │
│                                                                             │
│  isaacsim51-scene      ┌─ run_demo_scene.sh --world X1                      │
│  (this doc)            └─ isaacsim ... --exec scene_app.py                  │
│                              ├─ Pegasus PX4MavlinkBackend: TCP 4560 listener │
│                              ├─ ROS clock + LiDAR + state/TF ActionGraphs   │
│                              └─ cargo ROS 2 interface (`/cargo_bay/*`)       │
│                                                                             │
│  px4-sitl (Docker)     ┌─ PX4 SITL v1.16.2 (PX4MavlinkBackend ⇄ TCP 4560)   │
│                        └─ MicroXRCEAgent :8888 ↔ host FastDDS dom 45 ↔ /fmu/*│
└─────────────────────────────────────────────────────────────────────────────┘
```

## 这两个 daemon 的差别

| | `isaacsim51.service` | `isaacsim51-scene.service`(本文档) |
|---|---|---|
| ExecStart | `isaacsim isaacsim.exp.full.kit ...`(空 Kit) | `run_demo_scene.sh --world X1`(挂载 cargo scene) |
| PX4 链路 | ❌ 不挂 | ✅ Pegasus PX4MavlinkBackend 监听 TCP 4560 |
| `/fmu/*` 话题 | ❌ | ✅ 51 个 |
| `/cargo_bay/*` ROS 接口 | ❌ | ✅ |
| 默认 enable | ✅(被我们 `disable --now` 了) | ✅(本文档启用) |
| 用途 | M2 手画 ActionGraph / OmniGraph | 比赛链 / 接口验收 / 整机联调 |

两个 daemon **互斥** — 不要同时 start。`xvfb-isaac` / `x11vnc-isaac` 是共享基础设施,两边都能用。

## 常用操作

```bash
# 状态
sudo systemctl status isaacsim51-scene.service
sudo systemctl status isaacsim51.service xvfb-isaac.service x11vnc-isaac.service

# 启动 / 停 / 重启
sudo systemctl start   isaacsim51-scene.service
sudo systemctl restart isaacsim51-scene.service
sudo systemctl stop    isaacsim51-scene.service

# 开机自启 / 取消
sudo systemctl enable  isaacsim51-scene.service
sudo systemctl disable isaacsim51-scene.service

# 看日志(StandardOutput / StandardError 都 append 到这)
sudo tail -f /var/workspace/docker/isaac/logs/isaacsim51-scene.log

# Journalctl(包含 systemd 自己的 stdout/stderr + 状态切换)
sudo journalctl -u isaacsim51-scene.service -f
```

切回空 Kit daemon(比如手画 ActionGraph):

```bash
sudo systemctl stop    isaacsim51-scene.service
sudo systemctl enable  --now isaacsim51.service
# 干完手画工作想换回场景:
sudo systemctl stop    isaacsim51.service
sudo systemctl enable  --now isaacsim51-scene.service
```

## 验收(scene ready 之后)

`scene_app.py` 跑到 `Cargo delivery scene is ready` 是预期结束点(之后等用户在
VNC 里按 Play)。整链验收用 `drone_interface_audit`:

```bash
# 0. 确认 VNC 里看到 Kit 窗口,VNC: localhost:5900,密码 robo2026

# 1. 在 VNC Kit 里按 Play(Manual Mode 默认不开自动 tick)。

# 2. host 启 navigation chain:
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py

# 3. 接口票据:
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p report_path:=/tmp/drone_interface_report.json
# 期望 ok=true
```

如果只是想验证 4560 / `/fmu/*` 起来,不用 Play,看下面:

```bash
ss -tlnp | grep ':4560'                           # Pegasus listener
ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic list | grep -c /fmu/
# → 51
docker logs --tail=10 px4-sitl | grep -E "Connected|time sync"
```

## 部署时踩过的坑(避免下次重犯)

### 1. **不要覆盖 `VK_DRIVER_FILES` / `VK_ICD_FILENAMES`**

```bash
# 错(老版本 run_demo_scene.sh 这么写):
VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json

# /etc/vulkan/icd.d/ 在本机是空的 —— 没有 symlink 到真 ICD。
# 真 NVIDIA Vulkan ICD 在 /usr/share/vulkan/icd.d/nvidia_icd.json。
# 覆盖变量后 vulkan loader 找不到 driver,Kit 退到软件路径,
# Xvfb 出来的就是黑屏,log 里能看到:
#   VkResult: ERROR_INCOMPATIBLE_DRIVER
#   vkCreateInstance failed. Vulkan 1.1 is not supported
```

**修法**:`run_demo_scene.sh` 不设这两个变量,让 vulkan loader 走默认搜索。`isaacsim51.service` 老 unit 也是这么工作的,这就是为什么老的 systemd-launched Kit 能看到 GPU。

如果你看到 log 出现 "ERROR_INCOMPATIBLE_DRIVER" 但 daemon 已 active,先确认:

```bash
ls -la /etc/vulkan/icd.d/                          # 应该是空
ls -la /usr/share/vulkan/icd.d/                    # 应该有 nvidia_icd.json
cat /var/workspace/docker/isaac/logs/isaacsim51-scene.log | grep -E "Vulkan|Driver Version"
# 应该看到:
#   Driver Version: 580.159.03    | Graphics API: Vulkan
#   GPU | Name  | Active | ...
```

### 2. **`OMNI_KIT_ACCEPT_EULA=YES` 必须设**

`run_demo_scene.sh` 的 env -i 块现在带 `OMNI_KIT_ACCEPT_EULA=YES`(我们加的)。
这个变量是 Isaac Sim 5.1 在 systemd-launch(stdin 是 /dev/null)场景下唯一能绕过
EULA 交互提示的办法。只设 `ACCEPT_EULA=Y` 不够,Kit 还是会等 stdin 输入然后 EOF 退出。

### 3. **`/usr/share/vulkan/icd.d/` 是真 ICD 路径,不是 `/etc/vulkan/icd.d/`**

跨发行版 / 跨容器镜像的话这个路径可能不一样。在你那台机器上跑:

```bash
find / -name 'nvidia_icd.json' 2>/dev/null
```

让 vulkan loader 走默认搜索(`/usr/share/vulkan/icd.d` 默认是 fallback 之一),不要手
写绝对路径。

### 4. **`scene_app.py` 的四个本地补丁**

`scene_app.py` 是项目 target 版本,本地 Pegasus 5.1 / Isaac 5.1 / 已加载
`/World/quadrotor`(sunray.usd) API 表面有 drift。我们补丁了 4 处:

| 文件 | 补丁 | 原因 |
|---|---|---|
| `Multirotor(...)` 调用 | 去掉 `attach_existing=True` kwarg,改 `init_pos/init_orientation` 默认占位 | Pegasus 5.1 的 Multirotor 不收 `attach_existing`;prim 实际 pose 由 sunray.usd 给 |
| `CargoBayRuntime._make_prim_editable` | 加 no-op stub | 2 个 call site 都有 `or child` / `or mesh` 兜底,no-op 等价于直接保留原 ref |
| `_vec3f` | 补单精度 Gf.Vec3f helper | `Usd.LocalPos0Attr` / xform `RotateXYZ` op 用单精度,`_vec3d` 不行 |
| `CargoBayRuntime.ensure_runtime_scope` | 真实现:`/Runtime` scope 缺失时 `UsdGeom.Scope.Define` | 给 `PAYLOAD_LOCK_JOINT_PATH` 留父 |
| `world.reset_async()` | 包 try/except | sunray.usd 自带的 articulation 与 Pegasus Multirotor 期望的结构不完全匹配,`is_homogeneous` 失败;不让它阻止后续 ROS 接口 / Play |

如果重装 Pegasus 或换 sunray.usd 源,这些补丁可能失效,需要重审。

### 5. **scene 起来后 Multirotor 的 `is_homogeneous` 失败是已知 race**

log 里:

```
Isaac World reset_async failed ('NoneType' object has no attribute 'is_homogeneous'); 
continuing without full Multirotor physics initialization
```

不影响 4560 / `/fmu/*`,但 Pegasus 端的电机物理反馈回路不全。完整 motor dynamics
需要重写 `/World/quadrotor` 的 articulation 结构或者改 scene_app.py 让 Pegasus 自带
USD 重新 spawn(`init_pos=spawn_world` 而不是默认占位)。

### 6. **`mission_autostart=false` 仍然生效**

`flight_supervisor` 默认不会解锁,要等 `competition_orchestrator_pkg` 发送
`/drone/navigation/mission_request`。验收分阶段(未解锁 → Offboard → 单目标 →
障碍 → 视觉 → 完整任务)按 `docs/runbooks/drone_navigation.md` §7。

## 文件位置

| 文件 | 用途 |
|---|---|
| `/etc/systemd/system/isaacsim51-scene.service` | scene daemon unit(含 RTX / window flags) |
| `/etc/systemd/system/isaacsim51-scene.service.d/99-tuning.conf` | OOM/cgroup/restart 限制 |
| `/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/run_demo_scene.sh` | 场景 launcher,被 unit 的 ExecStart 调 |
| `/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/scene_app.py` | 实际 scene 装配 + Pegasus 后端挂载 |
| `/var/workspace/docker/isaac/logs/isaacsim51-scene.log` | daemon stdout/stderr |
| `/var/workspace/docker/isaac/logs/isaacsim51.log` | 老 daemon 的 log(若重启 `isaacsim51.service` 会被覆盖) |

## RTX / window 调优(2026-07-18)

Tesla T4 有 16 GB VRAM,默认 Kit 配 `Performance` preset + DLSS 自动降采样,导致
RTX 内部只渲 320×240 然后 upscale,VRAM 占用 <0.1 GB,GPU 利用率 0%。把下面这些
flag 加到 systemd unit 的 `ExecStart` 透传给 isaacsim:

```
--/app/window/width=1920
--/app/window/height=1080
--/app/window/fullscreen=true          # 真正占满 Xvfb 1920x1080(否则被 WM 装饰压到 1900x1022)
--/app/window/decorations=false        # 配合 fullscreen
--/rtx/quality/dlssMode=0              # 关 DLSS,按 native 分辨率渲
--/rtx/quality/preset=MaxQuality       # Performance 改 MaxQuality
--/rtx/quality/aaMode=0                # native 渲 + 关 AA(native 分辨率下 AA 价值低)
--/renderer/skipWhileMinimized=false   # 焦点丢失也保持渲染
--/renderer/multiGpu/enabled=0         # 单 GPU 全力跑
--/renderer/multiGpu/autoEnable=0
```

调优效果(Tesla T4):

| | 默认 | 调优后 |
|---|---|---|
| 窗口 | 1900×1022 (WM 装饰吃了) | 1920×1080 fullscreen |
| VRAM | 97 MiB / 16 GB | ~4 GB / 16 GB |
| GPU util | 0% | 67%+ |
| RTX preset | Performance (320×240 internal) | MaxQuality (native) |

log 里 `rtx.postprocessing.plugin] DLSS increasing input dimensions: Render
resolution of (320, 240)` 是 **viewport 之外的某个 hidden preview pass**(scene_app.py
隐藏了 LiDAR visual 但保留了它的 render target)的输入尺寸告警,**不是**主 viewport
渲染分辨率。主 viewport 已经按 1920×1080 渲了。


## 双模式：Pegasus 比赛场景 vs SIH 主机侧验证

2026-07-20 起，比赛链验收需要 **两套互斥的物理后端**。

### 模式 A：Pegasus 比赛场景（默认，比赛主链）

- daemon `isaacsim51-scene.service` active。
- PX4 容器 `PX4_SIM_MODEL=gazebo-classic_iris`（`docker-compose.yml` 默认）。
- 需要 VNC 客户端在 :5900（密码 `robo2026`）按 Play 才能让 scene 跑物理。
- 适合：视频录制、最终比赛场景验收、`docs/runbooks/drone_navigation.md` §7
  分阶段放飞。

### 模式 B：PX4 SIH 主机侧验证（D-5 起的 host 链烟测）

不用 Pegasus 物理，跳过 `is_homogeneous` race。PX4 自己跑 `sihsim_quadx` 物理。
主机侧 4 节点链（`navigation.launch.py`）+ 接口审计可以端到端跑通：
起飞 → Offboard → 1.8m 悬停 → Land。

切换：
```bash
cd /var/workspace/docker/isaac/docker/px4
docker compose down
PX4_SIM_MODEL=sihsim_quadx docker compose up -d
```

切回 A：
```bash
cd /var/workspace/docker/isaac/docker/px4
docker compose down
docker compose up -d
```

注意：`direct-rotor`（`/drone0/control/rotor*/ref`）和 SIH/Pegasus 是**第三个
互斥模式**，由 `DRONE_BACKEND` 环境变量切换，跑 `direct_rotor_smoke_test` 时启用。

比赛当天要回 A 模式验收。SIH 模式下的验收**不算比赛证据**，但
`docs/runbooks/drone_navigation.md` §7 验收可以拆：SIH 验证 host 链正确，
Pegasus 验证视觉 / 避障 / 货物投放。
## 相关文档

- `docs/setup/env_setup.md` — 总环境方案(Isaac Sim 5.1, PegasusSimulator,
  px4_msgs release/1.16, ROS 2 Jazzy)
- `docs/setup/foxglove_setup.md` — Foxglove WebSocket 桥(`bridge_competition_pkg` 装的 systemd
  daemon 是独立常驻,与本 daemon 解耦)
- `docs/runbooks/drone_navigation.md` — 无人机全链启动 / 接口票据 / 标定 / 验收
- `docs/contracts/interface_contracts.md` — topic / action / frame / param 契约,真值源是
  `bridge_competition_pkg/interface_audit.py::DEFAULT_REQUIRED_TOPICS`
- `../contracts/interface_contracts.md` §"运行时票据" — `drone_interface_audit` 命令
- `src/px4_sitl_usage.md` — PX4 SITL 容器本身(`px4-sitl` docker compose)
