# isaac_ros2_control — Isaac Sim 5.1 × ROS2 控制 starter

你自己的 ROS2 控制代码包。直接在 `workspace/src/` 下 `ros2 pkg create` 加新包即可，
本包是给你一个"立刻能跑"的最小示范。

## 包含的节点

| 节点 | 干啥 |
|---|---|
| `cmd_vel_relay` | 订阅 `/cmd_vel` 并 log（验证 host→Isaac 链路通） |
| `tf_echo_bridge` | 订阅 `/tf` 并 log 所有 frame 关系（验证 Isaac→host 链路通） |

## 编译 & 跑

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isaac_ros2_control
source install/setup.bash

# 开两个终端：
ros2 run isaac_ros2_control cmd_vel_relay        # 终端 1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0.5},angular:{z:0.3}}" -r 10  # 终端 2

ros2 run isaac_ros2_control tf_echo_bridge       # 终端 3 (Isaac 端有 PublishTF 节点时才有 /tf)
```

## 加新包

```bash
cd /var/workspace/docker/isaac/workspace/src
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy geometry_msgs
cd ..
colcon build --packages-select my_pkg
```

## 前置条件（Isaac 端要做的）

启动 `isaac-windowed` 或新 `isaacsim51` 服务后，VNC 连进去：
1. `Window → Script Editor`，建场景 + 加机器人
2. `Create → OmniGraph → ROS2` 加 ROS2 节点：
   - `ROS2Context`（domain=45）
   - `ROS2SubscribeTwist` 接 `/cmd_vel`
   - `PublishTF` 发 `/tf`
   - `ROS2PublishClock` 发 `/clock`（如果需要）
3. `World.play()` 开仿真

> ROS_DOMAIN_ID / RMW / LD_LIBRARY_PATH 都在 `isaacsim51.service` 的 `Environment=` 里统一设了，
> **不要**在每个脚本里手动 export。