"""Runtime ROS graph audit for the frozen Isaac/PX4 interface ticket."""

from collections import deque
from functools import partial
import json
from pathlib import Path
import time
from typing import Dict, List

from bridge_competition_pkg.interface_contract import (
    evaluate_interface,
    observed_frequency_hz,
    resolve_actual_topic,
)
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


COMMON_REQUIRED_TOPICS = [
    '/clock',
    '/tf',
    '/drone0/state/pose',
    '/drone0/state/twist',
    '/drone0/state/twist_inertial',
    '/drone0/state/accel',
    '/drone0/sensors/imu',
    '/drone0/sensors/mag',
    '/drone0/sensors/gps',
    '/drone0/sensors/gps_twist',
    '/avoidance/lidar/pointcloud',
    '/drone_0_ego_odom',
    '/drone0/down_camera/color/image_raw',
    '/cargo_bay/command',
    '/cargo_bay/status',
]

PX4_REQUIRED_TOPICS = [
    *COMMON_REQUIRED_TOPICS,
    '/fmu/out/vehicle_odometry',
    '/fmu/out/vehicle_status',
    '/fmu/out/vehicle_command_ack',
    '/fmu/out/vehicle_land_detected',
    '/fmu/in/offboard_control_mode',
    '/fmu/in/trajectory_setpoint',
    '/fmu/in/vehicle_command',
]

DIRECT_ROTOR_REQUIRED_TOPICS = [
    *COMMON_REQUIRED_TOPICS,
    '/drone0/control/rotor0/ref',
    '/drone0/control/rotor1/ref',
    '/drone0/control/rotor2/ref',
    '/drone0/control/rotor3/ref',
]

DEFAULT_REQUIRED_TOPICS = PX4_REQUIRED_TOPICS


class InterfaceAudit(Node):
    def __init__(self) -> None:
        super().__init__('drone_interface_audit')
        self.declare_parameter('required_topics', DEFAULT_REQUIRED_TOPICS)
        self.declare_parameter('backend_mode', 'px4')
        self.declare_parameter('report_path', '')
        self.declare_parameter('audit_period', 2.0)
        backend_mode = str(self.get_parameter('backend_mode').value)
        if backend_mode not in {'px4', 'direct_rotor'}:
            raise ValueError("backend_mode must be 'px4' or 'direct_rotor'")
        self._required = (
            list(DIRECT_ROTOR_REQUIRED_TOPICS)
            if backend_mode == 'direct_rotor'
            else list(self.get_parameter('required_topics').value)
        )
        self._require_fmu_writer = backend_mode == 'px4'
        self._report_path = str(self.get_parameter('report_path').value)
        self._sample_times = {}
        self._frame_ids = {}
        self._header_stamps = {}
        self._topic_subscriptions = {}
        self._publisher = self.create_publisher(
            String, '/drone/navigation/interface_audit', 10
        )
        self.create_timer(
            float(self.get_parameter('audit_period').value),
            self._audit,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    @staticmethod
    def _qos(endpoint) -> Dict[str, str]:
        profile = endpoint.qos_profile
        return {
            'reliability': profile.reliability.name,
            'durability': profile.durability.name,
            'history': profile.history.name,
            'depth': str(profile.depth),
        }

    def _audit(self) -> None:
        graph_types = dict(self.get_topic_names_and_types())
        self._create_observers(graph_types)
        resolved = {
            name: resolve_actual_topic(name, graph_types) for name in self._required
        }
        topics: Dict[str, dict] = {}
        for name in self._required:
            actual = resolved[name]
            publishers = self.get_publishers_info_by_topic(actual)
            subscribers = self.get_subscriptions_info_by_topic(actual)
            topics[name] = {
                'required_name': name,
                'actual_name': actual,
                'types': graph_types.get(actual, []),
                'observed_frequency_hz': round(
                    observed_frequency_hz(list(self._sample_times.get(actual, []))), 3
                ),
                'frame_ids': sorted(self._frame_ids.get(actual, set())),
                'last_header_stamp': self._header_stamps.get(actual),
                'publishers': [
                    {
                        'node': self._node_path(endpoint),
                        'qos': self._qos(endpoint),
                    }
                    for endpoint in publishers
                ],
                'subscribers': [
                    {
                        'node': self._node_path(endpoint),
                        'qos': self._qos(endpoint),
                    }
                    for endpoint in subscribers
                ],
            }
        # The evaluate_interface helper does its own resolution, but we pass it
        # node-name maps keyed by the resolved name so its checks line up.
        publisher_nodes = {
            name: [entry['node'] for entry in topics[name]['publishers']]
            for name in self._required
        }
        subscriber_nodes = {
            name: [
                entry['node'] for entry in topics[name]['subscribers']
                if entry['node'] != '/drone_interface_audit'
            ]
            for name in self._required
        }
        summary = evaluate_interface(
            self._required,
            graph_types,
            publisher_nodes,
            subscriber_nodes,
            require_fmu_writer=self._require_fmu_writer,
        )
        report = {
            **summary,
            'topics': topics,
        }
        encoded = json.dumps(report, ensure_ascii=False, sort_keys=True)
        message = String()
        message.data = encoded
        self._publisher.publish(message)
        if self._report_path:
            path = Path(self._report_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        if not summary['unique_fmu_writer']:
            self.get_logger().error(
                '/fmu/in topics must each have exactly one /trajectory_executor writer'
            )
        elif not summary['ok']:
            self.get_logger().warn(
                'interface ticket failed: '
                f"missing={len(summary['missing'])} "
                f"unpublished={len(summary['unpublished'])} "
                f"disconnected_commands={len(summary['disconnected_commands'])}"
            )

    @staticmethod
    def _node_path(endpoint) -> str:
        namespace = endpoint.node_namespace.rstrip('/')
        return f'{namespace}/{endpoint.node_name}' if namespace else f'/{endpoint.node_name}'

    def _create_observers(self, graph_types: Dict[str, List[str]]) -> None:
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for required_name in self._required:
            actual_name = resolve_actual_topic(required_name, graph_types)
            if actual_name in self._topic_subscriptions or not graph_types.get(actual_name):
                continue
            try:
                message_type = get_message(graph_types[actual_name][0])
                self._topic_subscriptions[actual_name] = self.create_subscription(
                    message_type,
                    actual_name,
                    partial(self._observe_message, actual_name),
                    qos,
                )
            except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as exception:
                self.get_logger().warn(
                    f'cannot observe {required_name} (resolved {actual_name}) frequency/frame: {exception}'
                )

    def _observe_message(self, topic: str, message) -> None:
        samples = self._sample_times.setdefault(topic, deque(maxlen=200))
        samples.append(time.monotonic())
        header = getattr(message, 'header', None)
        if header is None:
            return
        if header.frame_id:
            self._frame_ids.setdefault(topic, set()).add(header.frame_id)
        self._header_stamps[topic] = {
            'sec': header.stamp.sec,
            'nanosec': header.stamp.nanosec,
        }

def main(args=None) -> None:
    rclpy.init(args=args)
    node = InterfaceAudit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
