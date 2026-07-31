"""Assert one direct rotor produces a measured Isaac rigid-body response."""

import json
import math
import os
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from bridge_competition_pkg.interface_contract import (
    direct_rotor_output_allowed,
)


def vector_distance(left, right) -> float:
    """Return Euclidean distance between equal-length vectors."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def quaternion_angle(left, right) -> float:
    """Return shortest angular distance between two XYZW quaternions."""
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    if left_norm == 0.0 or right_norm == 0.0:
        raise ValueError('quaternion must be non-zero')
    dot = abs(
        sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm)
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def angular_velocity_extrema(samples) -> dict:
    """Return signed FLU angular-velocity extrema for rotor mapping checks."""
    if not samples:
        raise ValueError('at least one angular-velocity sample is required')
    axes = tuple(zip(*(sample['angular_velocity'] for sample in samples)))
    return {
        'min_angular_velocity_flu_rad_s': tuple(min(axis) for axis in axes),
        'max_angular_velocity_flu_rad_s': tuple(max(axis) for axis in axes),
    }


class RotorMotionProbe(Node):
    """Pulse exactly one rotor and verify pose, attitude, or rate responds."""

    def __init__(self) -> None:
        """Configure the gated publishers and best-effort state inputs."""
        super().__init__('direct_rotor_motion_probe')
        self.declare_parameter('enabled', False)
        self.declare_parameter('rotor_index', 0)
        self.declare_parameter('secondary_rotor_index', -1)
        self.declare_parameter('rotor_speed_rad_s', 600.0)
        self.declare_parameter('use_rotor_speed_vector', False)
        self.declare_parameter(
            'rotor_speed_vector_rad_s',
            [0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('pulse_seconds', 0.5)
        self.declare_parameter('pre_pulse_seconds', 0.0)
        self.declare_parameter('pulse_uses_sim_time', False)
        self.declare_parameter('publish_period_seconds', 0.05)
        self.declare_parameter('discovery_timeout_seconds', 8.0)
        self.declare_parameter('position_threshold_m', 0.0002)
        self.declare_parameter('attitude_threshold_rad', 0.002)
        self.declare_parameter('angular_speed_threshold_rad_s', 0.05)

        enabled = bool(self.get_parameter('enabled').value)
        backend_mode = os.environ.get('DRONE_BACKEND', '').strip()
        if not direct_rotor_output_allowed(enabled, backend_mode):
            raise RuntimeError(
                'refusing rotor output: both -p enabled:=true and '
                'DRONE_BACKEND=direct_rotor are required'
            )

        self._rotor_index = int(self.get_parameter('rotor_index').value)
        if self._rotor_index not in range(4):
            raise ValueError('rotor_index must be in [0, 3]')
        requested_speed = float(self.get_parameter('rotor_speed_rad_s').value)
        self._speed = min(650.0, max(0.0, requested_speed))
        if bool(self.get_parameter('use_rotor_speed_vector').value):
            requested_speeds = tuple(
                float(value)
                for value in self.get_parameter(
                    'rotor_speed_vector_rad_s'
                ).value
            )
            if len(requested_speeds) != 4:
                raise ValueError('rotor_speed_vector_rad_s must contain 4 values')
            self._active_speeds = tuple(
                min(650.0, max(0.0, speed)) for speed in requested_speeds
            )
            self._active_rotors = tuple(
                index
                for index, speed in enumerate(self._active_speeds)
                if speed > 0.0
            )
            if not self._active_rotors:
                raise ValueError('rotor speed vector must activate a rotor')
        else:
            secondary_rotor_index = int(
                self.get_parameter('secondary_rotor_index').value
            )
            if secondary_rotor_index == -1:
                self._active_rotors = (self._rotor_index,)
            elif (
                secondary_rotor_index not in range(4)
                or secondary_rotor_index == self._rotor_index
            ):
                raise ValueError(
                    'secondary_rotor_index must be -1 or a different index in [0, 3]'
                )
            else:
                self._active_rotors = (
                    self._rotor_index,
                    secondary_rotor_index,
                )
            self._active_speeds = tuple(
                self._speed if index in self._active_rotors else 0.0
                for index in range(4)
            )
        self._pulse = min(
            1.0,
            max(0.1, float(self.get_parameter('pulse_seconds').value)),
        )
        self._pre_pulse = min(
            2.0,
            max(0.0, float(self.get_parameter('pre_pulse_seconds').value)),
        )
        self._pulse_uses_sim_time = bool(
            self.get_parameter('pulse_uses_sim_time').value
        )
        self._publish_period = min(
            1.0,
            max(
                0.01,
                float(self.get_parameter('publish_period_seconds').value),
            ),
        )
        self._discovery_timeout = max(
            1.0,
            float(self.get_parameter('discovery_timeout_seconds').value),
        )
        self._position_threshold = float(
            self.get_parameter('position_threshold_m').value
        )
        self._attitude_threshold = float(
            self.get_parameter('attitude_threshold_rad').value
        )
        self._angular_speed_threshold = float(
            self.get_parameter('angular_speed_threshold_rad_s').value
        )

        self._rotor_publishers = [
            self.create_publisher(
                Float64,
                f'/drone0/control/rotor{index}/ref',
                10,
            )
            for index in range(4)
        ]
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._pose = None
        self._twist = None
        self._sim_time = None
        self._samples = []
        self.create_subscription(
            Clock,
            '/clock',
            self._on_clock,
            sensor_qos,
        )
        self.create_subscription(
            PoseStamped,
            '/drone0/state/pose',
            self._on_pose,
            sensor_qos,
        )
        self.create_subscription(
            TwistStamped,
            '/drone0/state/twist',
            self._on_twist,
            sensor_qos,
        )

    def _on_pose(self, message: PoseStamped) -> None:
        self._pose = message.pose
        self._record_sample()

    def _on_twist(self, message: TwistStamped) -> None:
        self._twist = message.twist
        self._record_sample()

    def _on_clock(self, message: Clock) -> None:
        self._sim_time = (
            float(message.clock.sec)
            + float(message.clock.nanosec) * 1e-9
        )

    def _record_sample(self) -> None:
        if self._pose is None or self._twist is None:
            return
        position = self._pose.position
        orientation = self._pose.orientation
        angular = self._twist.angular
        self._samples.append(
            {
                'position': (position.x, position.y, position.z),
                'quaternion': (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
                'angular_speed': math.sqrt(
                    angular.x * angular.x
                    + angular.y * angular.y
                    + angular.z * angular.z
                ),
                'angular_velocity': (angular.x, angular.y, angular.z),
            }
        )

    def _publish(self, active_speed: float) -> None:
        for index, publisher in enumerate(self._rotor_publishers):
            speed = (
                self._active_speeds[index]
                if active_speed > 0.0
                else 0.0
            )
            publisher.publish(Float64(data=speed))

    def _spin_for(self, duration: float, active_speed: float) -> None:
        if self._pulse_uses_sim_time:
            self._spin_for_sim_time(duration, active_speed)
            return
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self._publish(active_speed)
                next_publish = now + self._publish_period
            rclpy.spin_once(self, timeout_sec=0.01)

    def _spin_for_sim_time(self, duration: float, active_speed: float) -> None:
        if self._sim_time is None:
            raise RuntimeError('simulation clock is unavailable')
        start_sim_time = self._sim_time
        wall_deadline = time.monotonic() + max(30.0, duration * 200.0)
        next_publish = 0.0
        while self._sim_time - start_sim_time < duration:
            if time.monotonic() >= wall_deadline:
                raise RuntimeError('simulation clock stalled during rotor pulse')
            if self._sim_time < start_sim_time:
                start_sim_time = self._sim_time
            now = time.monotonic()
            if now >= next_publish:
                self._publish(active_speed)
                next_publish = now + self._publish_period
            rclpy.spin_once(self, timeout_sec=0.01)

    def _wait_for_graph_and_state(self) -> None:
        deadline = time.monotonic() + self._discovery_timeout
        while time.monotonic() < deadline:
            self._publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            active_publishers_ready = all(
                self._rotor_publishers[index].get_subscription_count() > 0
                for index in self._active_rotors
            )
            clock_ready = (
                not self._pulse_uses_sim_time or self._sim_time is not None
            )
            if self._samples and active_publishers_ready and clock_ready:
                return
        raise RuntimeError(
            'timed out waiting for Isaac state and the selected '
            'rotor subscriber'
        )

    def run(self) -> dict:
        """Run one pulse, emit metrics, and fail if no motion is measured."""
        try:
            self._wait_for_graph_and_state()
            if self._pre_pulse > 0.0:
                self._spin_for(self._pre_pulse, 0.0)
            baseline = self._samples[-1]
            self._samples = []
            self._spin_for(self._pulse, self._speed)
            self._spin_for(0.5, 0.0)
            if not self._samples:
                raise RuntimeError(
                    'no state samples received during rotor pulse'
                )

            metrics = {
                'rotor': self._rotor_index,
                'active_rotors': self._active_rotors,
                'pulse_rad_s': self._speed,
                'rotor_speeds_rad_s': self._active_speeds,
                'pulse_seconds': self._pulse,
                'pulse_time_basis': (
                    'simulation' if self._pulse_uses_sim_time else 'wall'
                ),
                'baseline_position': baseline['position'],
                'final_position': self._samples[-1]['position'],
                'max_position_delta_m': max(
                    vector_distance(baseline['position'], sample['position'])
                    for sample in self._samples
                ),
                'max_attitude_delta_rad': max(
                    quaternion_angle(
                        baseline['quaternion'], sample['quaternion']
                    )
                    for sample in self._samples
                ),
                'max_angular_speed_rad_s': max(
                    sample['angular_speed'] for sample in self._samples
                ),
            }
            metrics.update(angular_velocity_extrema(self._samples))
            metrics['passed'] = (
                metrics['max_position_delta_m'] >= self._position_threshold
                or metrics['max_attitude_delta_rad']
                >= self._attitude_threshold
                or metrics['max_angular_speed_rad_s']
                >= self._angular_speed_threshold
            )
            print(json.dumps(metrics, indent=2))
            if not metrics['passed']:
                raise RuntimeError(
                    f'rotor {self._rotor_index} caused no measurable response'
                )
            return metrics
        finally:
            self._zero_all(repeat=10)

    def _zero_all(self, repeat: int = 1) -> None:
        for _ in range(repeat):
            for publisher in self._rotor_publishers:
                publisher.publish(Float64(data=0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def main(args=None) -> None:
    """Run the explicitly armed direct-rotor motion probe."""
    rclpy.init(args=args)
    node = None
    try:
        node = RotorMotionProbe()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
