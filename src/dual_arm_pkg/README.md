# dual_arm_pkg

> **Owner: A**  
> **比赛模块: M5（双臂驱动 + 双夹爪）**  
> **日期: 2026-07-17 起**

## 这是什么

Mercury X1 双臂轮式机器人的 ROS2 控制包。提供：

- 双 6-DOF 臂的 JointState 驱动
- 双夹爪的同步 action server
- 完整的 ground-state 状态机（OBSERVE → DETECT → ... → GROUND_DONE）
- （可选）cuRobo 运动规划集成（+4 加分项）

## 接口（与外部唯一的沟通方式）

### 发布
- `/hand_command_left` / `/hand_command_right` (sensor_msgs/JointState)
- `/left_gripper_command` / `/right_gripper_command` (sensor_msgs/JointState)
- `/arena/ground/state` (std_msgs/String)

### 提供 Action
- `/demo_dual_gripper_command` (grasp_demo_interfaces/action/DualGripperCommand)

### 订阅
- `/arm_camera/{rgb,depth,camera_info}_{left,right}` ← C 桥
- `/demo_grasp/object_point_base` ← B（深度+mask+TF 后的基坐标系 3D 点）
- `/cmd_vel` (底盘，未来接 Nav2)

详见 [`/var/workspace/docker/isaac/workspace/docs/contracts/interface_contracts.md`](../docs/contracts/interface_contracts.md)。

## 编译

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dual_arm_pkg
source install/setup.bash
```

## 运行

```bash
# 1. 启动 Isaac 端（Kit VNC）
# 2. host 端拉起节点
ros2 launch dual_arm_pkg dual_arm_bringup.launch.py
```

## 工作分支

`dual_arm_a`（在 `/var/workspace/wt_a`）。

## 详细 TODO

见 `TODO.md`。

## 不要做

- ❌ 不要 import 其他 package 的代码
- ❌ 不要写 YOLOE 检测（那是 B 的）
- ❌ 不要写 OmniGraph 节点（那是 C 的）
- ❌ 不要改其他 package 的文件
