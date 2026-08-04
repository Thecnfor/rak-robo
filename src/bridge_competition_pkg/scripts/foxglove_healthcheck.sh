#!/usr/bin/env bash
# foxglove_healthcheck.sh — 探测 foxglove_bridge 是否在响应 WS 握手
#
# 设计：
#   * 探测方式：HTTP/1.1 GET 带最小合法 WebSocket 升级头，5 秒内拿到响应字节即视为存活
#     - 真实 WS 客户端 (Foxglove Studio) 会得到 HTTP/1.1 101
#     - 我们的探测缺 sec-websocket-protocol 头，会得到 HTTP/1.1 400（这也算存活）
#     - 关键判断：收到任意 HTTP 响应 (status 行) 就证明 worker 还在处理；连接被接受后
#       5 秒内 0 字节 = 挂死（典型症状：ASIO worker 卡死、socket fd 耗尽）
#   * 失败策略：连续 N 次失败才重启，避免网络抖动导致误重启
#     - 阈值通过环境变量 FOXGLOVE_FAIL_THRESHOLD 调整，默认 3
#   * 冷却：单次探测失败不会立刻重启；service RestartSec=5 会拉起新进程，
#     探测循环在下个 5 分钟再次确认
#
# 退出码：
#   0 = 健康（service 不动）
#   1 = 不健康且未达重启阈值（仅累加失败计数）
#   2 = 不健康且已达阈值（已执行 systemctl restart；调用方不必再动）
#
# 部署：由 foxglove-healthcheck.timer 每 5 分钟触发，独立 user systemd unit
# 日志：append 到 workspace/log/foxglove/healthcheck.log

set -u

WS="/var/workspace/docker/isaac/workspace"
LOG="${WS}/log/foxglove/healthcheck.log"
COUNTER="/tmp/foxglove_health_fail_count"
THRESHOLD="${FOXGLOVE_FAIL_THRESHOLD:-3}"
PROBE_TIMEOUT="${FOXGLOVE_PROBE_TIMEOUT:-5}"
TARGET_HOST="${FOXGLOVE_PROBE_HOST:-127.0.0.1}"
TARGET_PORT="${FOXGLOVE_PROBE_PORT:-8765}"

mkdir -p "$(dirname "$LOG")"

log() {
    local ts
    ts="$(date -u '+%F %T')"
    echo "[$ts] $*" >> "$LOG"
}

# 当前失败计数（文件不存在 / 非数字 = 0）
read_count() {
    [[ -f "$COUNTER" ]] || { echo 0; return; }
    local v
    v="$(cat "$COUNTER" 2>/dev/null || true)"
    [[ "$v" =~ ^[0-9]+$ ]] || { echo 0; return; }
    echo "$v"
}

write_count() {
    echo "$1" > "$COUNTER"
}

# 探测：用 Python 一次 recv，避开 curl 缺 ws 子协议的语义噪音
probe() {
    python3 - "$TARGET_HOST" "$TARGET_PORT" "$PROBE_TIMEOUT" <<'PY' 2>/dev/null
import socket, sys
host, port, timeout = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
try:
    s = socket.create_connection((host, port), timeout=timeout)
    s.sendall(
        b"GET / HTTP/1.1\r\n"
        b"Host: " + host.encode() + b":" + str(port).encode() + b"\r\n"
        b"Upgrade: websocket\r\n"
        b"Connection: Upgrade\r\n"
        b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        b"Sec-WebSocket-Version: 13\r\n"
        b"\r\n"
    )
    s.settimeout(timeout)
    data = s.recv(64)
    s.close()
    # 任意 HTTP/1.1 状态行 = worker 在响应
    sys.exit(0 if data.startswith(b"HTTP/1.") else 1)
except (socket.timeout, ConnectionRefusedError, OSError):
    sys.exit(2)
PY
}

state="$(probe)"
rc=$?

case "$rc" in
    0)
        # 健康：清零计数
        prev="$(read_count)"
        if [[ "$prev" -gt 0 ]]; then
            log "OK probe succeeded (reset counter from $prev to 0)"
        fi
        write_count 0
        exit 0
        ;;
    2)
        # TCP 直接拒接 — service 死了，systemd 会自己拉起来，我们只需记录
        log "WARN connection refused (service down, systemd RestartSec=5 should recover)"
        write_count 0  # 不算失败，因为 systemd 自己处理
        exit 1
        ;;
    *)
        # 收到部分字节但不像 HTTP / 5 秒超时 = 真正的挂死
        cur="$(read_count)"
        cur=$((cur + 1))
        write_count "$cur"
        log "FAIL probe rc=$rc (fail count $cur / threshold $THRESHOLD)"
        if [[ "$cur" -ge "$THRESHOLD" ]]; then
            log "ACTION restarting foxglove-bridge.service (after $cur consecutive failures)"
            systemctl --user restart foxglove-bridge.service
            # 重启后清零，让下个探测周期重新评估
            write_count 0
            exit 2
        fi
        exit 1
        ;;
esac