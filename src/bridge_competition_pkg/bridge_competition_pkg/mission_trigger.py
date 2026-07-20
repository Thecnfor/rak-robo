"""Publish a one-shot mission request to the supervisor to start the air phase.

The supervisor stays in `IDLE` until `/drone/navigation/mission_request=true`
arrives on its transient_local subscription. We match that QoS profile
so the message is delivered even if the supervisor is started after the
trigger.

Usage:
    source /opt/ros/jazzy/setup.bash
    source /var/workspace/docker/isaac/workspace/install/setup.bash
    export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 run bridge_competition_pkg mission_trigger
"""

from __future__ import annotations

import sys

import rclpy
import rclpy.qos
from rclpy.node import Node
from std_msgs.msg import Bool


_MISSION_QOS = rclpy.qos.QoSProfile(
    depth=1,
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
)


class _Trigger(Node):
    def __init__(self) -> None:
        super().__init__('mission_trigger')
        self._publisher = self.create_publisher(
            Bool, '/drone/navigation/mission_request', _MISSION_QOS)

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
