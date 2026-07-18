"""ROS 2 adapter for down-camera target detection."""

import math

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from .drop_target_detection import annotate_detection, detect_drop_target, DetectionConfig


class DroneTargetDetector(Node):
    def __init__(self) -> None:
        super().__init__('drone_target_detector')
        self.declare_parameter('image_topic', '/drone0/down_camera/color/image_raw')
        self.declare_parameter('min_area', 300.0)
        self.declare_parameter('min_circularity', 0.70)
        self.declare_parameter('morphology_kernel', 5)
        self.declare_parameter('center_threshold', 0.04)
        self._config = DetectionConfig(
            min_area=float(self.get_parameter('min_area').value),
            min_circularity=float(self.get_parameter('min_circularity').value),
            morphology_kernel=int(self.get_parameter('morphology_kernel').value),
        )
        self._center_threshold = float(self.get_parameter('center_threshold').value)
        self._bridge = CvBridge()
        self._offset_pub = self.create_publisher(
            Float32MultiArray, '/drone/drop_target_offset', 10
        )
        self._ready_pub = self.create_publisher(Bool, '/drone/drop_command', 10)
        self._debug_pub = self.create_publisher(Image, '/drone/drop_target_debug', 2)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, message: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        detection = detect_drop_target(image, self._config)
        offset = Float32MultiArray()
        ready = Bool()
        if detection is None:
            offset.data = [0.0, 0.0, 0.0, 0.0]
            ready.data = False
        else:
            offset.data = [
                float(detection.nx),
                float(detection.ny),
                float(detection.area_fraction),
                float(detection.radius),
            ]
            ready.data = math.hypot(detection.nx, detection.ny) <= self._center_threshold
        self._offset_pub.publish(offset)
        self._ready_pub.publish(ready)
        debug = self._bridge.cv2_to_imgmsg(
            annotate_detection(image, detection), encoding='bgr8'
        )
        debug.header = message.header
        self._debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneTargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
