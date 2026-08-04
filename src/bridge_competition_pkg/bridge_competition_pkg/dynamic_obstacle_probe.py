"""Fail-closed RTX LiDAR visibility gate for the dynamic-obstacle ticket."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from pathlib import Path
from typing import Iterable, Sequence


def _rotate_xyzw(point, quaternion):
    x, y, z = (float(value) for value in point)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    # Quaternion-vector rotation expanded as v + 2*w*(q x v) + 2*q x (q x v).
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def count_transformed_points_in_box(
    points: Iterable[Sequence[float]],
    *,
    transform: tuple[Sequence[float], Sequence[float]],
    center: Sequence[float],
    size: Sequence[float],
    padding: float = 0.03,
) -> int:
    """Count finite sensor-frame points inside a map-frame axis-aligned box."""
    translation, quaternion = transform
    half = tuple((float(value) / 2.0) + float(padding) for value in size)
    center = tuple(float(value) for value in center)
    count = 0
    for point in points:
        if len(point) < 3 or not all(math.isfinite(float(value)) for value in point[:3]):
            continue
        rotated = _rotate_xyzw(point[:3], quaternion)
        mapped = tuple(rotated[index] + float(translation[index]) for index in range(3))
        if all(abs(mapped[index] - center[index]) <= half[index] for index in range(3)):
            count += 1
    return count


def obstacle_visibility_passes(
    *,
    baseline_count: int,
    obstacle_count: int,
    minimum_points: int,
    minimum_increment: int,
) -> bool:
    return (
        obstacle_count >= minimum_points
        and obstacle_count - baseline_count >= minimum_increment
    )


_BOUNDS_PATTERN = re.compile(
    r"bounds_min=\(([^)]+)\)\s+bounds_max=\(([^)]+)\)"
)


def status_bounds_match(
    status: str,
    *,
    center: Sequence[float],
    size: Sequence[float],
    tolerance: float = 0.002,
) -> bool:
    """Reject acknowledgements for a different pre-authored test object."""
    match = _BOUNDS_PATTERN.search(status)
    if match is None:
        return False
    try:
        lower = tuple(float(value) for value in match.group(1).split(','))
        upper = tuple(float(value) for value in match.group(2).split(','))
    except ValueError:
        return False
    if len(lower) != 3 or len(upper) != 3:
        return False
    expected_lower = tuple(
        float(center[index]) - float(size[index]) / 2.0 for index in range(3)
    )
    expected_upper = tuple(
        float(center[index]) + float(size[index]) / 2.0 for index in range(3)
    )
    return all(
        abs(actual - expected) <= tolerance
        for actual, expected in zip(lower + upper, expected_lower + expected_upper)
    )


def run_probe(parsed) -> dict:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class ProbeNode(Node):
        def __init__(self):
            super().__init__('dynamic_obstacle_visibility_probe_node')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.command_publisher = self.create_publisher(
                String, '/drone/test/dynamic_obstacle_command', 10
            )
            self.create_subscription(
                String,
                '/drone/test/dynamic_obstacle_status',
                self._on_status,
                10,
            )
            self.create_subscription(
                PointCloud2,
                '/avoidance/lidar/pointcloud',
                self._on_cloud,
                qos_profile_sensor_data,
            )
            self.mode = 'baseline'
            self.status = ''
            self.counts = []
            self.tf_errors = []

        def _on_status(self, message):
            self.status = message.data

        def _on_cloud(self, message):
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map',
                    message.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                translation = tf.transform.translation
                rotation = tf.transform.rotation
                records = point_cloud2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True
                )
                points = (
                    (record['x'], record['y'], record['z'])
                    for record in records
                )
                count = count_transformed_points_in_box(
                    points,
                    transform=(
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                    ),
                    center=parsed.center,
                    size=parsed.size,
                    padding=parsed.padding,
                )
                self.counts.append(count)
            except Exception as exc:  # runtime evidence is preserved in JSON
                if len(self.tf_errors) < 10:
                    self.tf_errors.append(str(exc))

        def publish_command(self, command):
            self.command_publisher.publish(String(data=command))

    def collect(node, sample_count, deadline):
        start = len(node.counts)
        while rclpy.ok() and len(node.counts) - start < sample_count:
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.counts[start:]

    rclpy.init()
    node = ProbeNode()
    deadline = time.monotonic() + parsed.timeout
    baseline = []
    obstacle = []
    try:
        node.status = ''
        while rclpy.ok() and 'removed' not in node.status:
            if time.monotonic() >= deadline:
                raise RuntimeError('dynamic obstacle remove acknowledgement timeout')
            node.publish_command('remove')
            rclpy.spin_once(node, timeout_sec=0.25)

        settle_until = min(deadline, time.monotonic() + parsed.settle_seconds)
        while rclpy.ok() and time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.counts.clear()
        baseline = collect(node, parsed.samples, deadline)
        if len(baseline) < parsed.samples:
            raise RuntimeError('pointcloud/tf baseline timeout')

        node.status = ''
        while rclpy.ok() and 'spawned ' not in node.status:
            if time.monotonic() >= deadline:
                raise RuntimeError('dynamic obstacle spawn acknowledgement timeout')
            node.publish_command('spawn')
            rclpy.spin_once(node, timeout_sec=0.25)
        if not status_bounds_match(
            node.status, center=parsed.center, size=parsed.size
        ):
            raise RuntimeError(
                'dynamic obstacle acknowledgement geometry does not match probe'
            )

        # Do not accept a cloud captured before the RTX scene update reached a frame.
        settle_until = min(deadline, time.monotonic() + parsed.settle_seconds)
        while rclpy.ok() and time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.counts.clear()
        obstacle = collect(node, parsed.samples, deadline)
        if len(obstacle) < parsed.samples:
            raise RuntimeError('pointcloud/tf obstacle timeout')

        baseline_count = max(baseline)
        obstacle_count = max(obstacle)
        passed = obstacle_visibility_passes(
            baseline_count=baseline_count,
            obstacle_count=obstacle_count,
            minimum_points=parsed.minimum_points,
            minimum_increment=parsed.minimum_increment,
        )
        return {
            'passed': passed,
            'baseline_counts': baseline,
            'obstacle_counts': obstacle,
            'baseline_max': baseline_count,
            'obstacle_max': obstacle_count,
            'minimum_points': parsed.minimum_points,
            'minimum_increment': parsed.minimum_increment,
            'center': list(parsed.center),
            'size': list(parsed.size),
            'status': node.status,
            'tf_errors': node.tf_errors,
        }
    except Exception as exc:
        return {
            'passed': False,
            'error': str(exc),
            'baseline_counts': baseline,
            'obstacle_counts': obstacle,
            'status': node.status,
            'tf_errors': node.tf_errors,
        }
    finally:
        # Fail closed: park the acceptance object even when pointcloud or TF fails.
        for _ in range(5):
            node.publish_command('remove')
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--center', nargs=3, type=float, default=(5.5, -2.75, 1.8))
    parser.add_argument('--size', nargs=3, type=float, default=(0.6, 0.6, 1.0))
    parser.add_argument('--padding', type=float, default=0.03)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--minimum-points', type=int, default=3)
    parser.add_argument('--minimum-increment', type=int, default=2)
    parser.add_argument('--settle-seconds', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--output', type=Path)
    parsed = parser.parse_args(args)
    result = run_probe(parsed)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if parsed.output:
        parsed.output.parent.mkdir(parents=True, exist_ok=True)
        parsed.output.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
