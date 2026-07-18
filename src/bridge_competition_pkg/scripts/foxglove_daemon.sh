#!/usr/bin/env bash
# foxglove_daemon.sh — tmux 守护脚本
#
# 功能：在 tmux session `foxglove` 内循环重启 foxglove_bridge，
#       仅在不运行 host_bridge_bringup 时提供独立 Foxglove 调试服务。
#       比赛启动路径由 host bringup 管理唯一实例，禁止同时运行本脚本。
#
# 设计取舍：
#   - 不做 mtime 监听（按需手动重启，命令单一）
#   - tmux session 挂了就整个挂了，不会自我修复（systemd 不开）
#   - 日志在 workspace/log/foxglove/，提交前归档

set -e

WS="/var/workspace/docker/isaac/workspace"
LOG_DIR="${WS}/log/foxglove"
LOG_FILE="${LOG_DIR}/foxglove_bridge.log"

cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 启动日志
{
    echo "==================================================="
    echo "[$(date '+%F %T')] foxglove_daemon starting"
    echo "workspace: $WS"
    echo "log file:  $LOG_FILE"
    echo "==================================================="
} >> "$LOG_FILE"

# 循环：桥挂了就重启
while true; do
    echo "[$(date '+%F %T')] launching foxglove_bridge..." >> "$LOG_FILE"
    # 直接重定向（不要用 script | head，否则 SIGPIPE 会把 bridge 杀了）
    ros2 launch bridge_competition_pkg foxglove_bridge.launch.py \
        >> "$LOG_FILE" 2>&1 || true
    echo "[$(date '+%F %T')] bridge exited, restarting in 2s..." >> "$LOG_FILE"
    sleep 2
done
