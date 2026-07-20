"""Stub the ground-task completion signal so the air supervisor can advance.

The real ground task lives in `dual_arm_pkg` (M5, currently empty) and the
M6 perception stack. Until those nodes are built, the supervisor needs a
`/arena/ground/state` message containing `COMPLETE`, `SUCCESS`, or
`GROUND_DONE` to leave `IDLE`. This node publishes that one-shot signal.

Use the same QoS profile the supervisor subscribes with (transient local,
reliable) so the message is delivered to the supervisor's transient_local
cache and the supervisor immediately observes the value:

    ros2 run bridge_competition_pkg ground_state_sim

Accepts `--state {COMPLETE|SUCCESS|GROUND_DONE}` (default `COMPLETE`).
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


_VALID_STATES = {'COMPLETE', 'SUCCESS', 'GROUND_DONE'}


class _GroundStateSim(Node):
    def __init__(self, state: str) -> None:
        super().__init__('ground_state_sim')
        self._publisher = self.create_publisher(
            String, '/arena/ground/state',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._state = state
        self._timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self) -> None:
        message = String()
        message.data = self._state
        self._publisher.publish(message)
        self.get_logger().info(f'Published /arena/ground/state={self._state}')
        self._timer.cancel()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--state', choices=sorted(_VALID_STATES), default='COMPLETE')
    args = parser.parse_args()
    rclpy.init()
    try:
        sim = _GroundStateSim(args.state)
        for _ in range(20):
            rclpy.spin_once(sim, timeout_sec=0.1)
        sim.destroy_node()
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
