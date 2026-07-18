#!/usr/bin/env python3
"""Compatibility wrapper for the canonical runtime interface audit.

Prefer:
  ros2 run bridge_competition_pkg drone_interface_audit \
    --ros-args -p report_path:=/tmp/drone_interface_report.json
"""

import json
from pathlib import Path
import subprocess
import sys
import time


def main() -> None:
    report = Path('/tmp/drone_interface_report.json')
    process = subprocess.Popen([
        'ros2', 'run', 'bridge_competition_pkg', 'drone_interface_audit',
        '--ros-args', '-p', f'report_path:={report}',
    ])
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not report.exists():
            time.sleep(0.2)
    finally:
        process.terminate()
        process.wait(timeout=5)
    if not report.exists():
        print('interface audit did not produce a report', file=sys.stderr)
        raise SystemExit(2)
    result = json.loads(report.read_text(encoding='utf-8'))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
