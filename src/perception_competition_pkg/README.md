# perception_competition_pkg

> **Owner: B**
> **比赛模块: M6（视觉对准 + YOLOE 二次开发）+ 部分 M5 的感知链路**
> **日期: 2026-07-17 起**

## 这是什么

比赛用的视觉感知包。包含三部分：

1. **地面物料检测**：基于 YOLOE 二次开发，识别 pencil 等物料
2. **位姿估计**：深度+mask+TF，输出基坐标系下的 3D 位置和姿态
3. **无人机下视视觉对准**：识别地面圆心，输出归一化偏移

## 接口（与外部唯一的沟通方式）

### 发布
- `/demo_grasp/object_mask` (sensor_msgs/Image)
- `/demo_grasp/bbox` (std_msgs/Float32MultiArray: [x1, y1, x2, y2])
- `/demo_grasp/object_point_camera` (geometry_msgs/PointStamped)
- `/demo_grasp/long_axis_camera` (geometry_msgs/Vector3Stamped)
- `/drone/drop_target_offset` (std_msgs/Float32MultiArray: [nx, ny, area, radius])
- `/drone/drop_target_debug` (sensor_msgs/Image)

### 提供 Action
- `/demo_detect_object` (grasp_demo_interfaces/action/DetectObject)

### 订阅
- `/arm_camera/rgb_left` / `/arm_camera/rgb_right` ← C 桥
- `/arm_camera/depth_left` / `/arm_camera/depth_right` ← C 桥
- `/drone/down_camera/rgb` ← C 桥
- `/drone/down_camera/depth` ← C 桥

详见 [`/var/workspace/docker/isaac/workspace/docs/contracts/interface_contracts.md`](../docs/contracts/interface_contracts.md)。

## 编译

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select perception_competition_pkg
source install/setup.bash
```

## 运行

```bash
ros2 launch perception_competition_pkg perception_bringup.launch.py
```

## 工作分支

`perception_b`（在 `/var/workspace/wt_b`）。

## 详细 TODO

见 `TODO.md`。

## 不要做

- ❌ 不要 import 其他 package 的代码
- ❌ 不要控制机械臂（那是 A 的）
- ❌ 不要写 OmniGraph 节点（那是 C 的）
- ❌ 不要调无人机姿态（那是 D 的）
- ❌ 不要用 AprilTag / QR 码（比赛违规）

## YOLOE 模型

放在 `weights/yoloe-26l-seg.pt`（沿用 grasp_demo_pkg 的权重，**gitignored**）。

可选：自己训练一个小模型针对比赛物料（pencil 等），放到 `weights/` 下，运行时通过参数切换。
