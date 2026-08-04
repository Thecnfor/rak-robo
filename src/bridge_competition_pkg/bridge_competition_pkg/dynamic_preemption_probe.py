"""Isolated ROS integration probe for EGO active-trajectory preemption.

This is test tooling. It injects a synthetic map-frame obstacle into a private
point-cloud topic and never runs in competition bringup.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Sequence


def choose_obstacle_center(points: Sequence[Sequence[float]]) -> tuple[float, float, float]:
    """Choose an interior point on an accepted path, away from both endpoints."""
    if len(points) < 3:
        raise ValueError('accepted trajectory needs at least three points')
    start = points[0]
    end = points[-1]
    candidates = []
    for point in points[1:-1]:
        from_start = math.dist(point, start)
        from_end = math.dist(point, end)
        if from_start >= 0.5 and from_end >= 0.5:
            candidates.append((abs(from_start - from_end), point))
    if not candidates:
        raise ValueError('accepted trajectory has no safe interior injection point')
    selected = min(candidates, key=lambda item: item[0])[1]
    return tuple(float(value) for value in selected[:3])


def obstacle_cluster(center: Sequence[float]) -> list[tuple[float, float, float]]:
    """Return a dense 0.3 m cube of map-frame returns around the path."""
    cx, cy, cz = (float(value) for value in center)
    offsets = (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15)
    return [
        (cx + dx, cy + dy, cz + dz)
        for dx in offsets
        for dy in offsets
        for dz in offsets
    ]


def run_probe(parsed) -> dict:
    if os.environ.get('ROS_DOMAIN_ID', '0') == '45':
        return {
            'passed': False,
            'evidence_kind': 'synthetic_pointcloud_runtime_preemption',
            'error': 'refusing test injection on competition ROS_DOMAIN_ID=45',
        }
    import rclpy
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )
    from drone_navigation_pkg.msg import Trajectory
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String, UInt32

    executable_dir = (
        Path(get_package_prefix('drone_navigation_pkg'))
        / 'lib'
        / 'drone_navigation_pkg'
    )
    config = (
        Path(get_package_share_directory('drone_navigation_pkg'))
        / 'config'
        / 'navigation.yaml'
    )
    planner_command = [
        str(executable_dir / 'ego_local_planner'),
        '--ros-args',
        '--params-file',
        str(config),
        '-p',
        'use_sim_time:=false',
        '-r',
        '__node:=dynamic_preemption_test_planner_node',
        '-r',
        '/avoidance/lidar/pointcloud:=/drone/test/preemption_cloud',
    ]
    executor_command = [
        str(executable_dir / 'trajectory_executor'),
        '--ros-args',
        '--params-file',
        str(config),
        '-p',
        'use_sim_time:=false',
        '-r',
        '__node:=dynamic_preemption_test_executor_node',
    ]
    planner = subprocess.Popen(
        planner_command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    executor = subprocess.Popen(
        executor_command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class ProbeNode(Node):
        def __init__(self):
            super().__init__('dynamic_preemption_probe_node')
            self.odom_pub = self.create_publisher(
                Odometry, '/drone/navigation/odometry', 20
            )
            self.goal_pub = self.create_publisher(
                PoseStamped, '/drone/navigation/goal', transient
            )
            self.cloud_pub = self.create_publisher(
                PointCloud2,
                '/drone/test/preemption_cloud',
                qos_profile_sensor_data,
            )
            self.create_subscription(
                UInt32,
                '/drone/navigation/accepted_trajectory_id',
                self._on_accepted,
                transient,
            )
            self.create_subscription(
                Trajectory,
                '/drone/navigation/trajectory',
                self._on_trajectory,
                transient,
            )
            self.create_subscription(
                String,
                '/drone/navigation/planner_state',
                self._on_state,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.create_subscription(
                String,
                '/drone/navigation/executor_state',
                self._on_executor_state,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.first_trajectory_id = None
            self.preempting_trajectory_id = None
            self.pending_injection_center = None
            self.injection_center = None
            self.states = []
            self.executor_states = []
            self.accepted_ids = set()

        def _on_state(self, message):
            if not self.states or self.states[-1] != message.data:
                self.states.append(message.data)
                self.states = self.states[-30:]

        def _on_executor_state(self, message):
            if not self.executor_states or self.executor_states[-1] != message.data:
                self.executor_states.append(message.data)
                self.executor_states = self.executor_states[-30:]

        def _on_accepted(self, message):
            accepted_id = int(message.data)
            self.accepted_ids.add(accepted_id)
            if accepted_id == self.first_trajectory_id:
                self.injection_center = self.pending_injection_center

        def _on_trajectory(self, message):
            if self.first_trajectory_id is None:
                path = [
                    (point.position.x, point.position.y, point.position.z)
                    for point in message.points
                ]
                self.pending_injection_center = choose_obstacle_center(path)
                self.first_trajectory_id = int(message.trajectory_id)
            elif (
                message.trajectory_id > self.first_trajectory_id
                and message.preemption_reason == Trajectory.PREEMPTION_OBSTACLE
            ):
                self.preempting_trajectory_id = int(message.trajectory_id)

        def publish_inputs(self):
            stamp = self.get_clock().now().to_msg()
            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = 'map'
            odometry.pose.pose.position.x = 0.0
            odometry.pose.pose.position.y = 0.0
            odometry.pose.pose.position.z = 1.0
            odometry.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odometry)

            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = 'map'
            goal.pose.position.x = 2.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 1.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

            header = Header(stamp=stamp, frame_id='map')
            if self.injection_center is None:
                points = [(100.0, 100.0, 100.0)]
            else:
                points = obstacle_cluster(self.injection_center)
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    rclpy.init()
    node = ProbeNode()
    deadline = time.monotonic() + parsed.timeout
    obstacle_acceptance = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if planner.poll() is not None or executor.poll() is not None:
                raise RuntimeError('isolated planner/executor exited before probe completed')
            node.publish_inputs()
            rclpy.spin_once(node, timeout_sec=0.05)
            obstacle_acceptance = (
                node.preempting_trajectory_id is not None and
                node.preempting_trajectory_id in node.accepted_ids and
                any(
                    f'TRAJECTORY_ACCEPTED id={node.preempting_trajectory_id} '
                    'reason=obstacle' in state
                    for state in node.executor_states
                )
            )
            if obstacle_acceptance and any(
                'ACTIVE_OBSTACLE_REPLAN' in state for state in node.states
            ):
                break
        passed = obstacle_acceptance and any(
            'ACTIVE_OBSTACLE_REPLAN' in state for state in node.states)
        return {
            'passed': passed,
            'evidence_kind': 'synthetic_pointcloud_runtime_preemption',
            'first_trajectory_id': node.first_trajectory_id,
            'preempting_trajectory_id': node.preempting_trajectory_id,
            'injection_center': node.injection_center,
            'planner_states': node.states,
            'executor_states': node.executor_states,
            'accepted_trajectory_ids': sorted(node.accepted_ids),
        }
    except Exception as exc:
        return {
            'passed': False,
            'evidence_kind': 'synthetic_pointcloud_runtime_preemption',
            'error': str(exc),
            'planner_states': node.states,
        }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        planner.terminate()
        executor.terminate()
        try:
            output, _ = planner.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            planner.kill()
            output, _ = planner.communicate()
        try:
            executor_output, _ = executor.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            executor.kill()
            executor_output, _ = executor.communicate()
        if parsed.planner_log:
            parsed.planner_log.parent.mkdir(parents=True, exist_ok=True)
            parsed.planner_log.write_text(
                '[planner]\n' + output + '\n[executor]\n' + executor_output,
                encoding='utf-8',
            )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--planner-log', type=Path)
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
