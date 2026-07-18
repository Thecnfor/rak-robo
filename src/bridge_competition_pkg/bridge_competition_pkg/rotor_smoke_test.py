"""Explicitly armed, low-speed direct-rotor interface test."""

import os
import time

from bridge_competition_pkg.interface_contract import direct_rotor_output_allowed
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class RotorSmokeTest(Node):
    def __init__(self) -> None:
        super().__init__('direct_rotor_smoke_test')
        self.declare_parameter('enabled', False)
        self.declare_parameter('rotor_speed_rad_s', 50.0)
        self.declare_parameter('pulse_seconds', 0.5)
        enabled = bool(self.get_parameter('enabled').value)
        backend_mode = os.environ.get('DRONE_BACKEND', '').strip()
        if not direct_rotor_output_allowed(enabled, backend_mode):
            raise RuntimeError(
                'refusing rotor output: both -p enabled:=true and '
                'DRONE_BACKEND=direct_rotor are required'
            )
        requested_speed = float(self.get_parameter('rotor_speed_rad_s').value)
        self._speed = min(100.0, max(0.0, requested_speed))
        self._pulse = min(1.0, max(0.1, float(self.get_parameter('pulse_seconds').value)))
        self._publishers = [
            self.create_publisher(Float64, f'/drone0/control/rotor{index}/ref', 10)
            for index in range(4)
        ]

    def run(self) -> None:
        try:
            for rotor_index, publisher in enumerate(self._publishers):
                self.get_logger().warn(
                    f'pulsing rotor {rotor_index} at {self._speed:.1f} rad/s'
                )
                deadline = time.monotonic() + self._pulse
                while time.monotonic() < deadline:
                    message = Float64()
                    message.data = self._speed
                    publisher.publish(message)
                    rclpy.spin_once(self, timeout_sec=0.0)
                    time.sleep(0.02)
                self._zero_all()
                time.sleep(0.2)
        finally:
            self._zero_all(repeat=10)

    def _zero_all(self, repeat: int = 1) -> None:
        message = Float64()
        message.data = 0.0
        for _ in range(repeat):
            for publisher in self._publishers:
                publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotorSmokeTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
