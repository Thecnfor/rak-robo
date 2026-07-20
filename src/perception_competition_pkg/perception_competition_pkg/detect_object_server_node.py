"""Action server wrapping YOLOE + depth-pose detection.

Bridges the `/demo_detect_object` action (`grasp_demo_interfaces`) to
`yoloe_detector_node` (publishes `/demo_grasp/{bbox, label, confidence,
object_mask}`) and `depth_pose_estimator_node` (publishes
`/demo_grasp/{object_point_camera, long_axis_camera}`). Returns a single
detection with the strongest bbox in the requested class set, projected
to the camera frame.

This is the `M2.5` server the teaching pick_place_state_machine calls
before /demo_plan_to_pose. Real YOLOE weights are not yet deployed; the
server reads the stub topic publications and returns the synthetic
detection emitted by `yoloe_detector_node`.

Usage:
    ros2 run perception_competition_pkg detect_object_server_node
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray, String
from grasp_demo_interfaces.action import DetectObject


class _DetectObjectServer(Node):
    def __init__(self) -> None:
        super().__init__('detect_object_server')
        self._action = ActionServer(
            self, DetectObject, '/demo_detect_object',
            execute_callback=self._execute,
            goal_callback=lambda req: rclpy.action.GoalResponse.ACCEPT,
            cancel_callback=lambda req: rclpy.action.CancelResponse.ACCEPT,
        )
        self._bbox: Optional[Float32MultiArray] = None
        self._label: Optional[str] = None
        self._point: Optional[PointStamped] = None
        self._axis: Optional[Vector3Stamped] = None
        self._conf: float = 0.0
        sensor_qos = qos_profile_sensor_data
        self.create_subscription(
            Float32MultiArray, '/demo_grasp/bbox', self._on_bbox, sensor_qos)
        self.create_subscription(
            String, '/demo_grasp/label', self._on_label, sensor_qos)
        self.create_subscription(
            PointStamped, '/demo_grasp/object_point_camera',
            self._on_point, sensor_qos)
        self.create_subscription(
            Vector3Stamped, '/demo_grasp/long_axis_camera',
            self._on_axis, sensor_qos)
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Float32, '/demo_grasp/confidence', self._on_conf, reliable_qos)
        self.get_logger().info(
            'detect_object_server ready: /demo_detect_object '
            '(bridges YOLOE + depth pose)')

    def _on_bbox(self, message: Float32MultiArray) -> None:
        self._bbox = message

    def _on_label(self, message: String) -> None:
        self._label = message.data

    def _on_point(self, message: PointStamped) -> None:
        self._point = message

    def _on_axis(self, message: Vector3Stamped) -> None:
        self._axis = message

    def _on_conf(self, message: Float32) -> None:
        self._conf = float(message.data)

    def _execute(self, goal_handle) -> DetectObject.Result:
        import time
        request = goal_handle.request
        timeout = float(request.timeout) if request.timeout > 0.0 else 5.0
        target_classes = set(request.target_classes)
        self.get_logger().info(
            f'detect_object: classes={list(target_classes)} '
            f'threshold={request.confidence_threshold:.2f}')
        deadline = time.time() + timeout
        while time.time() < deadline and self._bbox is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        result = DetectObject.Result()
        if (self._bbox is None or self._label is None or
                self._point is None or self._axis is None):
            result.success = False
            result.message = 'no detection within timeout'
            goal_handle.abort()
            return result
        if (target_classes and
                self._label not in target_classes and
                'all' not in target_classes):
            result.success = False
            result.message = f'label {self._label!r} not in target_classes'
            goal_handle.abort()
            return result
        if self._conf < float(request.confidence_threshold):
            result.success = False
            result.message = (
                f'confidence {self._conf:.2f} below threshold '
                f'{request.confidence_threshold:.2f}')
            goal_handle.abort()
            return result
        result.success = True
        result.message = f'ok (confidence {self._conf:.2f})'
        result.detected_class = self._label or ''
        result.object_position = self._point
        result.normal = Vector3Stamped()
        result.normal.header = self._point.header
        result.normal.vector.z = 1.0
        result.long_axis = self._axis
        result.confidence = self._conf
        goal_handle.succeed()
        return result


def main() -> int:
    rclpy.init()
    try:
        node = _DetectObjectServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
