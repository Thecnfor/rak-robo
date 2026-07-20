"""Publish a one-shot mission request to the supervisor to start the air phase.

Used by the M9.4 flight acceptance run. The supervisor stays in `IDLE` until
`/drone/navigation/mission_request=true` arrives, and the `mission_autostart`
flag in `navigation.yaml` only gates this from the YAML side. Once the ground
task is complete and this fires, the supervisor moves through
`PREFLIGHT → ARMING → TAKEOFF → EGO_TRANSIT → TARGET_SEARCH → VISUAL_ALIGN →
DROP_HOLD → RETURN → LAND → COMPLETE`.

Usage:
    source /opt/ros/jazzy/setup.bash
    source /var/workspace/docker/isaac/workspace/install/setup.bash
    export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 run bridge_competition_pkg mission_trigger
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class _Trigger(Node):
    def __init__(self) -> None:
        super().__init__('mission_trigger')
        self._publisher = self.create_publisher(
            Bool, '/drone/navigation/mission_request', 10)

    def fire(self) -> None:
        message = Bool()
        message.data = True
        self._publisher.publish(message)
        self.get_logger().info(
            'Published mission_request=true; supervisor will run if '
            'mission_autostart is enabled and ground task is complete.')


def main() -> int:
    rclpy.init()
    try:
        trigger = _Trigger()
        # spin briefly so the publisher is discovered by DDS before we publish
        for _ in range(5):
            rclpy.spin_once(trigger, timeout_sec=0.1)
        trigger.fire()
        for _ in range(5):
            rclpy.spin_once(trigger, timeout_sec=0.1)
        trigger.destroy_node()
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
