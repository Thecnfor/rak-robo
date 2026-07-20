"""Print the live state of every node in the competition UAV chain.

Designed for VNC-on-call debugging. Run from a sourced workspace:

    source /opt/ros/jazzy/setup.bash
    source /var/workspace/docker/isaac/workspace/install/setup.bash
    export ROS_DOMAIN_ID=45 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 run bridge_competition_pkg chain_status

Output is a single page of the latest published state from each
`/drone/navigation/*` topic plus the supervisor's `/drone/navigation/state`.
`ok=true` means every required topic is present and `/fmu/in/*` has exactly
one publisher. Use this to answer the question "did the chain come up?"
without opening Foxglove.
"""

from __future__ import annotations

import json
import sys
import time
from typing import Dict, List

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from bridge_competition_pkg.interface_contract import (
    evaluate_interface,
    resolve_actual_topic,
)


DEFAULT_REQUIRED = [
    '/clock',
    '/tf',
    '/drone0/state/pose',
    '/drone0/sensors/imu',
    '/avoidance/lidar/pointcloud',
    '/drone_0_ego_odom',
    '/drone0/down_camera/color/image_raw',
    '/cargo_bay/command',
    '/cargo_bay/status',
    '/fmu/out/vehicle_odometry',
    '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_command_ack',
    '/fmu/out/vehicle_land_detected',
    '/fmu/in/offboard_control_mode',
    '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command',
]


class _LatestSubscriber(Node):
    def __init__(self, topic: str, qos: QoSProfile) -> None:
        super().__init__(f'_chain_status_{topic.replace("/", "_")}')
        self.latest: object | None = None
        self.create_subscription(
            String, topic, self._on_message, qos)

    def _on_message(self, message: String) -> None:
        self.latest = message.data


class _BoolSubscriber(Node):
    def __init__(self, topic: str, qos: QoSProfile) -> None:
        super().__init__(f'_chain_status_bool_{topic.replace("/", "_")}')
        self.latest: object | None = None
        self.create_subscription(Bool, topic, self._on_message, qos)

    def _on_message(self, message: Bool) -> None:
        self.latest = message.data


def _evaluate(ros_distro: str | None = None) -> dict:
    rclpy.init()
    try:
        node = rclpy.create_node('chain_status_audit')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        graph_types = dict(node.get_topic_names_and_types())
        publishers_by_topic: Dict[str, List[str]] = {}
        subscribers_by_topic: Dict[str, List[str]] = {}
        for required in DEFAULT_REQUIRED:
            actual = resolve_actual_topic(required, graph_types)
            pubs = node.get_publishers_info_by_topic(actual)
            subs = node.get_subscriptions_info_by_topic(actual)
            publishers_by_topic[required] = [
                endpoint.node_name for endpoint in pubs]
            subscribers_by_topic[required] = [
                endpoint.node_name for endpoint in subs if
                endpoint.node_name != node.get_name()]
        summary = evaluate_interface(
            DEFAULT_REQUIRED,
            graph_types,
            publishers_by_topic,
            subscribers_by_topic,
            require_fmu_writer=True,
        )
        snapshot = {'topics': {}}
        string_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        bool_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        live_topics = {
            '/drone/navigation/px4_status': string_qos,
            '/drone/navigation/state': string_qos,
            '/drone/navigation/executor_state': string_qos,
            '/drone/navigation/planner_state': string_qos,
            '/drone/navigation/landed': bool_qos,
        }
        observers = []
        for topic, qos in live_topics.items():
            if topic in graph_types:
                if 'Bool' in graph_types[topic][0]:
                    sub = _BoolSubscriber(topic, qos)
                else:
                    sub = _LatestSubscriber(topic, qos)
                executor.add_node(sub)
                observers.append((topic, sub))
        deadline = time.time() + 1.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
        for topic, sub in observers:
            snapshot['topics'][topic] = sub.latest
        for sub in observers:
            sub.destroy_node()
        node.destroy_node()
        return {'summary': summary, 'live': snapshot}
    finally:
        rclpy.shutdown()


def main() -> int:
    result = _evaluate()
    summary = result['summary']
    live = result['live']
    print('=== INTERFACE CONTRACT ===')
    print(json.dumps(summary, indent=2, sort_keys=True))
    print()
    print('=== LIVE STATE ===')
    for topic in sorted(live['topics']):
        value = live['topics'][topic]
        if value is None:
            print(f'{topic}: (no message yet)')
        else:
            print(f'{topic}: {value}')
    return 0 if summary.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
