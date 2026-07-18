"""
Foxglove tmux daemon console entry point.

用法:
    ros2 run bridge_competition_pkg foxglove_daemon            # 帮助
    ros2 run bridge_competition_pkg foxglove_daemon start      # 启动 tmux session
    ros2 run bridge_competition_pkg foxglove_daemon stop       # 杀掉 tmux session
    ros2 run bridge_competition_pkg foxglove_daemon restart    # Ctrl+C 触发 daemon 自动重启
    ros2 run bridge_competition_pkg foxglove_daemon status     # 看 tmux + 8765 + 日志末尾
    ros2 run bridge_competition_pkg foxglove_daemon attach     # 进 tmux（Ctrl+B d 退出）
"""

import argparse
from pathlib import Path
import subprocess
import sys

WORKSPACE = Path('/var/workspace/docker/isaac/workspace')
SESSION = 'foxglove'
SCRIPT = WORKSPACE / 'src' / 'bridge_competition_pkg' / 'scripts' / 'foxglove_daemon.sh'
LOG_FILE = WORKSPACE / 'log' / 'foxglove' / 'foxglove_bridge.log'


def run(cmd, check=False, capture=False):
    """Run a command; return CompletedProcess or raise."""
    return subprocess.run(cmd, check=check, text=True,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.PIPE if capture else None)


def session_exists() -> bool:
    return run(['tmux', 'has-session', '-t', SESSION], capture=True).returncode == 0


def cmd_start(args) -> int:
    if session_exists():
        print(f'tmux session "{SESSION}" 已存在；如需重启请用 restart')
        return 1
    if not SCRIPT.exists():
        print(f'找不到脚本: {SCRIPT}', file=sys.stderr)
        return 1
    SCRIPT.chmod(0o755)
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['tmux', 'new-session', '-d', '-s', SESSION, '-c', str(WORKSPACE),
           f'bash {SCRIPT}']
    run(cmd, check=True)
    print(f'✅ tmux session "{SESSION}" 已启动，日志: {LOG_FILE}')
    return 0


def cmd_stop(args) -> int:
    if not session_exists():
        print(f'tmux session "{SESSION}" 不存在')
        return 1
    run(['tmux', 'kill-session', '-t', SESSION], check=True)
    print(f'✅ tmux session "{SESSION}" 已停')
    return 0


def cmd_restart(args) -> int:
    if not session_exists():
        print(f'tmux session "{SESSION}" 不存在；用 start 起一个')
        return 1
    run(['tmux', 'send-keys', '-t', SESSION, 'C-c'], check=True)
    print('✅ 已发送 Ctrl+C，daemon 会在 ~3 秒内拉起新实例')
    return 0


def cmd_status(args) -> int:
    # tmux session
    if session_exists():
        print(f'🟢 tmux session "{SESSION}" alive')
    else:
        print(f'🔴 tmux session "{SESSION}" not running')

    # 8765 port
    res = run(['ss', '-tlnp'], capture=True)
    if '8765' in res.stdout:
        pid_line = [line for line in res.stdout.splitlines() if '8765' in line]
        if pid_line:
            print(f'🟢 8765 listening: {pid_line[0].strip()}')
    else:
        print('🔴 8765 not listening')

    # 日志末尾
    if LOG_FILE.exists():
        print(f'\n--- 最近 5 行日志 ({LOG_FILE}) ---')
        tail = run(['tail', '-n', '5', str(LOG_FILE)], capture=True)
        print(tail.stdout or '(空)')
    else:
        print(f'\n⚠️ 日志文件不存在: {LOG_FILE}')

    return 0


def cmd_attach(args) -> int:
    if not session_exists():
        print(f'tmux session "{SESSION}" 不存在；用 start 起一个')
        return 1
    # 用 execvp 让 attach 接管当前终端
    try:
        subprocess.run(['tmux', 'attach', '-t', SESSION], check=True)
    except KeyboardInterrupt:
        pass
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        prog='foxglove_daemon',
        description='Foxglove tmux 守护控制台（端口 8765 固定）',
    )
    sub = parser.add_subparsers(dest='action')

    sub.add_parser('start', help='启动 tmux session（开机/重启后用）')
    sub.add_parser('stop', help='杀掉 tmux session')
    sub.add_parser('restart', help='Ctrl+C 触发 daemon 自动重启（改完 YAML 用）')
    sub.add_parser('status', help='看 tmux + 8765 + 日志末尾')
    sub.add_parser('attach', help='进 tmux 实时看日志（Ctrl+B d 退出）')

    args = parser.parse_args()
    handlers = {
        'start': cmd_start,
        'stop': cmd_stop,
        'restart': cmd_restart,
        'status': cmd_status,
        'attach': cmd_attach,
    }
    if not args.action:
        parser.print_help()
        return 0
    return handlers[args.action](args)


if __name__ == '__main__':
    sys.exit(main())
