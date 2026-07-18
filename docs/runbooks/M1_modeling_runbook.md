# M1 建模逐步操作手册（A 主导 · VNC 上 Isaac Kit GUI）

> 目标：在 Isaac Sim 5.1 中装配出 Mercury X1 + Sunray150 + air_fpv_box + 物料 + 相机的完整场景，并清点出所有 prim 路径，整理成 `prim_path_registry.yaml`。

---

## 前置条件

- VNC 已连入中心服务器（密码 `robo2026`，端口 `5900`）
- isaacsim51.service 已 `active (running)`（`systemctl status isaacsim51`）
- 打开 Kit：`Window → Script Editor` 已就绪

---

## Step 1（5 分钟）— 加载场景

在 Kit Script Editor 跑：
```python
from omni.isaac.core import World
world = World()
scene_usd = "/var/workspace/docker/isaac/scenes/X1_assets/assets/full_worlds/Collected_X1_race_scene/X1_race_scene.usd"
# 用 Reference 加载（不要直接 Open，免得 Scene Prim 错乱）
import omni.kit.commands
omni.kit.commands.execute("CreateReference",
    usd_context=omni.usd.get_context(),
    path="/World/RaceScene",
    asset_path=scene_usd)
world.play()
```

**检查**：Stage Tree 出现 `/World/RaceScene/{layout,X1,...}`。截图给 D 留档。

---

## Step 2（10 分钟）— 检查 Mercury X1 关节

```python
from pxr import Usd
stage = Usd.Stage.Open("<用 File → Save As 临时保存的 USD 路径>")
```

在 Stage Tree 找到 X1 装配节点（一般在 `/World/RaceScene/.../X1`），展开后记录：
- 底盘 4 轮 joint 名（如 `front_left_wheel_joint`）
- left_arm 6 个 joint 名（j1..j6）
- right_arm 6 个 joint 名
- 两组 gripper joint 名

把结果填到 `docs/prim_path_registry.yaml` 模板的 `mercury_x1:` 段。

---

## Step 3（10 分钟）— 装上臂相机

Create → Camera 两次，作为 left arm 和 right arm 的子节点。`/World/.../X1/left_arm/.../arm_camera_left` 和 `arm_camera_right`。

关键 prop：
- focal_length ~ 24mm
- resolution 640×480
- 30 fps

---

## Step 4（10 分钟）— 装 Sunray + air_fpv_box

```python
omni.kit.commands.execute("CreateReference",
    path="/World/quadrotor",
    asset_path="/var/workspace/docker/isaac/scenes/bobac_assets/robot/robot/sunray.usd")

omni.kit.commands.execute("CreateReference",
    path="/World/quadrotor/mounted_cargo_bay",
    asset_path="/var/workspace/docker/isaac/scenes/X1_assets/assets/tools/air_fpv_box.usd")
```

确认 `/World/quadrotor/mounted_cargo_bay/transparent_cargo_bay/{cargo_body, joints/{left_door_joint, bottom_door_joint}}` 全部存在（参照 config.py 的命名）。

---

## Step 5（10 分钟）— Sunray 下视相机 + X1 LiDAR

- 下视相机：放在 Sunray 腹部下方 0.4m，朝下
- LiDAR：2D 雷达挂在 X1 底盘上方 0.15m
  - 配置：max_range 8m、min_angle -π、max_angle π、rot_freq 10Hz

---

## Step 6（5 分钟）— 物料摆放

Create → Mesh → cylinder 之类，在物料区摆 3-5 个目标物料（先 pencil，与 YOLOE classes 对齐）。

---

## Step 7（10 分钟）— 保存 USD

File → Save As：
```
/var/workspace/docker/isaac/scenes/environments/<team>_office_X1_full.usd
```

---

## Step 8（10 分钟）— 整理 prim_path_registry.yaml

D 复制模板 `docs/prim_path_registry.template.yaml`，按 Step 2-5 的真实路径填写。交付物路径：
```
docs/prim_path_registry.yaml
```

---

## 完成后验收清单

- [ ] `mercury_x1` 所有 joint 名记录
- [ ] left_arm_camera, right_arm_camera, drone_down_camera 路径记录
- [ ] LiDAR prim 路径记录
- [ ] `/World/quadrotor/mounted_cargo_bay/.../left_door_joint` 和 `bottom_door_joint` 路径记录
- [ ] `<team>_office_X1_full.usd` 已保存到 scenes/environments
- [ ] prim_path_registry.yaml 已交付
- [ ] 给 D：截图 + USD 路径 + YAML 路径

交给 C：`prim_path_registry.yaml` 一交付，C 就可以开 M2 了。
