"""Depth-pose estimator stub for the Mercury X1 arm camera.

The real pipeline reads `/arm_camera/depth + /arm_camera/camera_info +
/demo_grasp/bbox`, picks the median valid depth around the bounding-box
center, back-projects into camera-frame XYZ, and computes the image-plane
long axis with PCA over `/demo_grasp/object_mask`. This stub keeps the
**contract** alive so the downstream `tf_transform_demo_node` and
`plan_to_pose_node` can be wired before the real back-projection is
calibrated.

Contract (`grasp_demo_pkg/config/demo_params.yaml`):
    /arm_camera/depth                     ->  sensor_msgs/Image   (32FC1 m)
    /arm_camera/camera_info               ->  sensor_msgs/CameraInfo
    /demo_grasp/bbox                      ->  std_msgs/Float32MultiArray
    /demo_grasp/object_mask               ->  sensor_msgs/Image
    /demo_grasp/object_point_camera       ->  geometry_msgs/PointStamped
    /demo_grasp/long_axis_camera          ->  geometry_msgs/Vector3Stamped

Usage:
    ros2 run perception_competition_pkg depth_pose_estimator_node
    ros2 run perception_competition_pkg depth_pose_estimator_node --ros-args \
        -p simulated_distance:=1.2
"""

from __future__ import annotations

import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Header


class DepthPoseEstimator(Node):
    def __init__(self) -> None:
        super().__init__('depth_pose_estimator_node')
        self.declare_parameter('depth_topic', '/arm_camera/depth')
        self.declare_parameter('camera_info_topic', '/arm_camera/camera_info')
        self.declare_parameter('bbox_topic', '/demo_grasp/bbox')
        self.declare_parameter('simulated_distance', 1.2)
        self._point_pub = self.create_publisher(
            PointStamped, '/demo_grasp/object_point_camera', 10)
        self._axis_pub = self.create_publisher(
            Vector3Stamped, '/demo_grasp/long_axis_camera', 10)
        self._last_bbox: Optional[Float32MultiArray] = None
        self.create_subscription(
            Float32MultiArray, self.get_parameter('bbox_topic').value,
            self._on_bbox, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self._on_depth, qos_profile_sensor_data)
        self._timer = self.create_timer(0.5, self._publish_synthetic)
        self.get_logger().info(
            'Depth pose estimator running in stub mode; '
            'point sits at the bbox center in image plane.')

    def _on_bbox(self, message: Float32MultiArray) -> None:
        self._last_bbox = message

    def _on_info(self, _message: CameraInfo) -> None:
        return

    def _on_depth(self, _message: Image) -> None:
        return

    def _publish_synthetic(self) -> None:
        if self._last_bbox is None or len(self._last_bbox.data) < 4:
            return
        bbox = self._last_bbox.data
        cx = bbox[0] + bbox[2] / 2.0
        cy = bbox[1] + bbox[3] / 2.0
        distance = float(self.get_parameter('simulated_distance').value)
        point = PointStamped()
        point.header = Header()
        point.header.frame_id = 'arm_camera_optical_frame'
        point.point.x = (cx - 320.0) * 0.001 * distance
        point.point.y = (cy - 240.0) * 0.001 * distance
        point.point.z = distance
        self._point_pub.publish(point)
        axis = Vector3Stamped()
        axis.header = point.header
        # Default long axis: along the +X image direction.
        axis.vector.x = math.cos(0.0)
        axis.vector.y = 0.0
        axis.vector.z = 0.0
        self._axis_pub.publish(axis)


def main() -> int:
    rclpy.init()
    try:
        node = DepthPoseEstimator()
        rclpy.spin(node)
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
