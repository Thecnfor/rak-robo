# 无人机全链路运行手册

## 当前交付状态

代码、消息、launch、Docker 外部模拟器模式和离线测试已落地。正式飞行仍有四个必须
完成的门槛：取得可达的指定 EGO commit 并移植 raycast/LBFGS、接口票据、投放区
`drop_search_pose`、下视相机朝向/PID。
在这些票据通过前保持 `mission_autostart=false`，不要解锁。

## 1. 环境

```bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

版本固定为 Isaac Sim 5.1、ROS 2 Jazzy、PX4 v1.16.2、`px4_msgs release/1.16`。

## 2. Isaac/Pegasus 模式

比赛主链：

```bash
export DRONE_BACKEND=px4
/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/run_demo_scene.sh --world X1
```

逐电机烟测：

```bash
export DRONE_BACKEND=direct_rotor
/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/run_demo_scene.sh --world X1
```

不要同时运行 systemd 的 GUI Isaac 和上述独立 launcher。若 GUI 服务正在使用，应在
Kit Script Editor 执行：

```python
import os, runpy
os.environ["CARGO_DELIVERY_WORLD"] = "X1"
os.environ["DRONE_BACKEND"] = "px4"
runpy.run_path(
    "/var/workspace/docker/isaac/scenes/active/scripts/integrated_runtime/scene_app.py",
    run_name="__main__",
)
```

也可由操作者明确停止 GUI 服务后再启动独立进程。场景内 `px4_autolaunch=false`，PX4
生命周期只归 Docker 管。

## 3. PX4 Docker

```bash
cd /var/workspace/docker/isaac/docker/px4
docker compose build
docker compose up -d
docker compose logs -f px4
```

Isaac 未加载时日志停在
`Waiting for simulator to accept connection on TCP port 4560` 是正确的安全状态；连接成功
后启动传感器、飞控和 uXRCE-DDS，届时 host 才会出现 `/fmu/*`。

独立 PX4 SIH 烟测与比赛主链分离：

```bash
PX4_SIM_MODEL=sihsim_quadx docker compose up -d --force-recreate
```

SIH 不能作为比赛飞行验收证据。

## 4. 接口票据

比赛场景播放、PX4 连接后执行：

```bash
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args \
  -p report_path:=/tmp/drone_interface_report.json \
  -p backend_mode:=px4
```

报告必须 `ok=true`；报告会记录实测频率、frame 和时间戳，还需人工核对电机映射。
direct 模式执行：

```bash
ros2 run bridge_competition_pkg direct_rotor_smoke_test \
  --ros-args -p enabled:=true -p rotor_speed_rad_s:=60.0
ros2 run bridge_competition_pkg drone_interface_audit \
  --ros-args -p backend_mode:=direct_rotor
```

direct 测试只允许低速、机体固定/螺旋桨安全状态；程序结束会向四电机发零。完成后必须
重启场景回 `DRONE_BACKEND=px4`。

## 5. 标定参数

编辑 `src/drone_navigation_pkg/config/navigation.yaml` 后重建：

- `px4_map_origin`：PX4 local `(0,0,0)` 对应的 Isaac 世界 ENU 出生点；当前 X1 初值
  `[4.55,-0.38,1.13]`。
- `drop_search_pose`：从最终 USD/比赛实景提取，当前 `[0,0,1.8]` 是占位。
- `takeoff_height` 和 `virtual_ceiling`：按净空校准。
- 相机必须确认 `/drone0/down_camera/color/image_raw` 是正下视；移动目标到图像四个方向，
  验证视觉速度符号后再调 `visual_kp`。
- `allow_manual_flight_actions` 和 `allow_manual_door_actions` 比赛默认必须为 `false`；只有
  独立人工接口测试才临时开启，LAND/ABORT 不受该开关限制。

## 6. Host bringup

```bash
cd /var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
```

bringup 启动四个导航节点、视觉检测、地空协调和接口审计；Foxglove（8765）由独立
tmux daemon 常驻。默认不会自动解锁；只有 `/arena/ground/state` 为
`COMPLETE|SUCCESS|GROUND_DONE` 后 orchestrator 才发一次任务请求。

## 7. 分阶段放飞

严格按下列顺序，每项保存 rosbag/PX4 日志：

1. 未解锁：传感器、坐标和接口报告。
2. Offboard：预流、切模式、解锁、1.8 m 起飞、30 s 定点、Land。
3. 单目标：0.5 m/s 到空旷目标并返航。
4. 障碍：静态、新出现障碍、局部死路。
5. 视觉：粗搜索位、稳定 0.8 s、水平速度 <0.05 m/s、开底舱。
6. 完整任务：关侧门→起飞→避障→对准→投放→返航→降落。

任何 odometry/点云超过 0.3 s 会 HOLD，超过 1.0 s 或 PX4 failsafe 请求 Land。

## 8. 验收证据

- 10 次连续完整流程，无碰撞、坠机、重置或人工干预。
- 投放至少 90% 静止落点距圆心 ≤0.2 m。
- 保存 `/tmp/drone_interface_report.json`、rosbag、ULog、状态机日志、规划轨迹和全程视频。
- 接口/标定不通过时，不能以“节点已启动”替代比赛验收。
