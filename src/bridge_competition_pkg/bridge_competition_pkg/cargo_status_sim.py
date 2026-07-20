"""Stub /cargo_bay/status so the flight supervisor's PREFLIGHT gate can
open when Pegasus's cargo door publisher is silent.

Real status messages come from the scene's cargo bay runtime. Until
the cargo runtime is wired (race condition in scene_app.py during the
is_homogeneous failure), this node publishes a status string that the
supervisor's onCargoStatus callback can match on.

The simulator mirrors what Pegasus would emit once the door primitives
respond:
- left_closed: as soon as the supervisor publishes left_close.
- bottom_opened payload_released: after a bottom_open is published.
- left_opened bottom_opened: idle state.

Usage:
    ros2 run bridge_competition_pkg cargo_status_sim
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class _CargoStatusSim(Node):
    def __init__(self) -> None:
        super().__init__('cargo_status_sim')
        self._publisher = self.create_publisher(
            String, '/cargo_bay/status', 10)
        self._subscription = self.create_subscription(
            String, '/cargo_bay/command', self._on_command, 10)
        self._last_status = 'left_closed bottom_opened'
        self._timer = self.create_timer(1.0, self._tick)

    def _on_command(self, message: String) -> None:
        cmd = message.data.strip()
        if cmd == 'left_close':
            self._last_status = 'left_closed bottom_opened'
        elif cmd == 'left_open':
            self._last_status = 'left_opened bottom_opened'
        elif cmd == 'bottom_open':
            self._last_status = 'left_closed bottom_opened payload_released'
        elif cmd == 'bottom_close':
            self._last_status = 'left_closed bottom_closed'
        self.get_logger().debug(
            f'cargo command="{cmd}" -> status="{self._last_status}"')

    def _tick(self) -> None:
        message = String()
        message.data = self._last_status
        self._publisher.publish(message)


def main() -> int:
    rclpy.init()
    try:
        sim = _CargoStatusSim()
        executor = SingleThreadedExecutor()
        executor.add_node(sim)
        executor.spin()
        sim.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
