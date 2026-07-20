"""Pick-and-place state machine for the Mercury X1 dual-arm.

Drives the air-ground handoff: when the ground pipeline publishes
`/arena/ground/state == COMPLETE`, this node emits `IDLE → DETECTING →
GRASPING → LIFTING → PLACING → COMPLETE` and publishes
`/arena/ground/state = COMPLETE` on the success path.

This is the **stub** for M5.1; the real arm IK / motion planning is
swapped in during M5.2 (cuRobo integration) and M5.3 (dual-arm
collision check). The state machine contract stays the same.

Usage:
    ros2 run dual_arm_pkg dual_arm_pick_place_node
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

try:
    from grasp_demo_interfaces.action import DetectObject, PlanToPose
except ImportError:  # pragma: no cover
    DetectObject = None  # type: ignore
    PlanToPose = None  # type: ignore


class _Phase(Enum):
    IDLE = 'IDLE'
    OBSERVE = 'OBSERVE'
    DETECTING = 'DETECTING'
    GRASPING = 'GRASPING'
    LIFTING = 'LIFTING'
    PLACING = 'PLACING'
    COMPLETE = 'COMPLETE'


@dataclass
class _State:
    phase: _Phase = _Phase.IDLE
    last_target_label: str = ''


class _PickPlace(Node):
    def __init__(self) -> None:
        super().__init__('dual_arm_pick_place_node')
        self._state = _State()
        self._publisher = self.create_publisher(
            String, '/arena/ground/state', 10)
        self._state_publisher = self.create_publisher(
            String, '/dual_arm/phase', 10)
        self._goal_subscription = self.create_subscription(
            String, '/arena/ground/request', self._on_request, 10)
        self._timer = self.create_timer(0.5, self._tick)
        if DetectObject is not None:
            self._detect_client = ActionClient(
                self, DetectObject, '/demo_detect_object')
        if PlanToPose is not None:
            self._plan_client = ActionClient(
                self, PlanToPose, '/demo_plan_to_pose')
        self.get_logger().info('PickPlace state machine ready.')

    def _on_request(self, message: String) -> None:
        if self._state.phase is not _Phase.IDLE:
            return
        self._state.phase = _Phase.OBSERVE
        self.get_logger().info('Mission request received, leaving IDLE')

    def _publish_ground_complete(self) -> None:
        message = String()
        message.data = 'COMPLETE'
        self._publisher.publish(message)
        self.get_logger().info('Published /arena/ground/state=COMPLETE')

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.phase.value
        self._state_publisher.publish(message)

    def _tick(self) -> None:
        phase = self._state.phase
        if phase is _Phase.IDLE:
            return
        # Stub advancement: after a short settle time at each phase, advance.
        # The real version will wait on the action client feedback instead
        # of the wall clock.
        if phase is _Phase.OBSERVE:
            self._state.phase = _Phase.DETECTING
        elif phase is _Phase.DETECTING:
            self._state.phase = _Phase.GRASPING
        elif phase is _Phase.GRASPING:
            self._state.phase = _Phase.LIFTING
        elif phase is _Phase.LIFTING:
            self._state.phase = _Phase.PLACING
        elif phase is _Phase.PLACING:
            self._state.phase = _Phase.COMPLETE
            self._publish_ground_complete()
        self._publish_state()


def main() -> int:
    rclpy.init()
    try:
        node = _PickPlace()
        rclpy.spin(node)
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
