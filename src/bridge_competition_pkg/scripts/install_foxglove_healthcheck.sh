#!/usr/bin/env bash
# install_foxglove_healthcheck.sh — 一次性安装 foxglove-bridge 健康探测
#
# 装两个 systemd --user unit：
#   foxglove-healthcheck.service  oneshot，跑探测脚本
#   foxglove-healthcheck.timer    每 5 分钟触发，OnBootSec=2min
#
# 依赖：foxglove-bridge.service 已在跑；脚本日志写在 workspace/log/foxglove/
#
# 用法：
#   bash src/bridge_competition_pkg/scripts/install_foxglove_healthcheck.sh
# 卸载：
#   systemctl --user disable --now foxglove-healthcheck.timer
#   rm ~/.config/systemd/user/foxglove-healthcheck.{service,timer}
#   systemctl --user daemon-reload

set -euo pipefail

WS="/var/workspace/docker/isaac/workspace"
SCRIPT_SRC="${WS}/src/bridge_competition_pkg/scripts/foxglove_healthcheck.sh"
# 装到 XDG 用户目录（systemd --user unit 不需要 sudo 也能跑）
SCRIPT_DST="${HOME}/.local/bin/foxglove_healthcheck.sh"
UNIT_DIR="${HOME}/.config/systemd/user"

[[ -f "$SCRIPT_SRC" ]] || { echo "❌ 找不到 $SCRIPT_SRC"; exit 1; }

# 1. 复制脚本到 /usr/local/bin（systemd unit 路径稳定）
install -m 0755 "$SCRIPT_SRC" "$SCRIPT_DST"
echo "✅ 已安装 $SCRIPT_DST"

# 2. 写 systemd units
mkdir -p "$UNIT_DIR"

cat > "${UNIT_DIR}/foxglove-healthcheck.service" <<'EOF'
[Unit]
Description=Foxglove bridge health probe (active probe + auto-restart)
Documentation=file:///var/workspace/docker/isaac/workspace/src/bridge_competition_pkg/scripts/foxglove_healthcheck.sh
After=foxglove-bridge.service
Wants=foxglove-bridge.service

[Service]
Type=oneshot
# 探测失败时会重启 foxglove-bridge.service，需要 reload 权限
ExecStart=%h/.local/bin/foxglove_healthcheck.sh
# 不让探测脚本卡住 timer
TimeoutStartSec=15
# 输出走 syslog，便于 journalctl 集中看
StandardOutput=journal
StandardError=journal
EOF

cat > "${UNIT_DIR}/foxglove-healthcheck.timer" <<'EOF'
[Unit]
Description=Periodic Foxglove bridge health probe
Documentation=https://www.freedesktop.org/software/systemd/man/systemd.timer.html

[Timer]
# 启动 2 分钟后首次跑，之后每 5 分钟
OnBootSec=2min
OnUnitActiveSec=5min
# 不允许漂移堆积
AccuracySec=10s
# 持久化：错过会补跑
Persistent=true
Unit=foxglove-healthcheck.service

[Install]
WantedBy=timers.target
EOF

echo "✅ 已写 systemd units: ${UNIT_DIR}/foxglove-healthcheck.{service,timer}"

# 3. 重新加载 + 启用 timer
systemctl --user daemon-reload
systemctl --user enable --now foxglove-healthcheck.timer

echo
echo "=== 当前状态 ==="
systemctl --user status foxglove-healthcheck.timer --no-pager | head -10
echo
systemctl --user list-timers foxglove-healthcheck.timer --no-pager | head -5