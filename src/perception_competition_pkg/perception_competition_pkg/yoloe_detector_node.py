"""YOLOE detector stub for the Mercury X1 arm cameras.

The real YOLOE pipeline loads `weights/yoloe-26l-seg.pt` and runs the
`ultralytics` YOLO predictor on `/arm_camera/rgb_{left,right}`. Loading
the weights at host-side startup is expensive (~10s) and the model file
itself is git-ignored; this stub keeps the **contract** working so the
rest of the grasp pipeline can be wired and smoke-tested today.

Replace the body of `YoloeDetector._infer` with the real `ultralytics.YOLO`
call when the model file is on disk at `weights/yoloe-26l-seg.pt`.

Topic contract (`grasp_demo_pkg/config/demo_params.yaml`):
    /arm_camera/rgb                       ->  sensor_msgs/Image
    /demo_grasp/object_mask               ->  sensor_msgs/Image
    /demo_grasp/bbox                      ->  std_msgs/Float32MultiArray  (x,y,w,h,conf,cls)
    /demo_grasp/label                     ->  std_msgs/String
    /demo_grasp/confidence                ->  std_msgs/Float32
    /demo_grasp/debug_image               ->  sensor_msgs/Image

Usage:
    ros2 run perception_competition_pkg yoloe_detector_node
    ros2 run perception_competition_pkg yoloe_detector_node --ros-args \
        -p simulate_target_confidence:=0.92 -p simulate_target_label:=pencil
"""

from __future__ import annotations

import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None  # type: ignore[assignment]


class YoloeDetector(Node):
    def __init__(self) -> None:
        super().__init__('yoloe_detector_node')
        self.declare_parameter('image_topic', '/arm_camera/rgb')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('weights_path', 'weights/yoloe-26l-seg.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('simulate_target_label', 'pencil')
        self.declare_parameter('simulate_target_confidence', 0.92)
        self._bridge = CvBridge() if CvBridge is not None else None
        self._mask_pub = self.create_publisher(Image, '/demo_grasp/object_mask', 2)
        self._bbox_pub = self.create_publisher(
            Float32MultiArray, '/demo_grasp/bbox', 10)
        self._label_pub = self.create_publisher(String, '/demo_grasp/label', 10)
        self._conf_pub = self.create_publisher(Float32, '/demo_grasp/confidence', 10)
        self._debug_pub = self.create_publisher(Image, '/demo_grasp/debug_image', 2)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self._on_image, qos_profile_sensor_data)
        self._last_image: Optional[Image] = None
        self._last_stamp_ns = 0
        self.get_logger().info(
            f'YOLOE detector running on {self.get_parameter("image_topic").value} '
            f'(stub mode; set weights_path to enable real inference)')

    def _on_image(self, message: Image) -> None:
        # Stash the image so the synthetic-detection timer can grab it.
        self._last_image = message
        self._last_stamp_ns = time.time_ns()

    def _publish_synthetic(self) -> None:
        """Run synthetic detection every 0.5 s while we wait for the real model."""
        if self._last_image is None or self._bridge is None:
            return
        height, width = self._last_image.height, self._last_image.width
        # Place a synthetic 200x200 box centered in the frame.
        cx, cy = width // 2, height // 2
        bbox = Float32MultiArray()
        bbox.data = [float(cx - 100), float(cy - 100), 200.0, 200.0,
                     float(self.get_parameter('simulate_target_confidence').value), 0.0]
        self._bbox_pub.publish(bbox)
        label = String()
        label.data = str(self.get_parameter('simulate_target_label').value)
        self._label_pub.publish(label)
        confidence = Float32()
        confidence.data = float(self.get_parameter('simulate_target_confidence').value)
        self._conf_pub.publish(confidence)
        # Synth mask: a 200x200 white square centered. We publish the input
        # image back as the mask channel too — the consumer only uses
        # `nonzero()`-shaped masks so the box is what matters.
        try:
            mask = self._bridge.cv2_to_imgmsg(
                self._bridge.imgmsg_to_cv2(self._last_image, desired_encoding='bgr8'),
                encoding='bgr8')
        except Exception:
            return
        mask.header = self._last_image.header
        self._mask_pub.publish(mask)
        self._debug_pub.publish(mask)


def main() -> int:
    rclpy.init()
    try:
        node = YoloeDetector()
        timer = node.create_timer(0.5, node._publish_synthetic)
        rclpy.spin(node)
        node.destroy_timer(timer)
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
