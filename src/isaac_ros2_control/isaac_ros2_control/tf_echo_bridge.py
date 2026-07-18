#!/usr/bin/env python3
"""订阅 Isaac Sim 发的 /tf，打印 robot frame 之间的最新变换。

Isaac 端：在场景里加 ROS2 OmniGraph `PublishTF` 节点就会发 /tf。
host 端：本节点订阅 /tf 并 log 所有 (parent → child) 关系，便于调试。

用法：
  ros2 run isaac_ros2_control tf_echo_bridge
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfEchoBridge(Node):
    def __init__(self):
        super().__init__('tf_echo_bridge')
        self.sub = self.create_subscription(
            TFMessage, '/tf', self.on_tf, 50)

    def on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.get_logger().info(
                f'{t.header.frame_id} -> {t.child_frame_id}: '
                f'x={t.transform.translation.x:.3f} '
                f'y={t.transform.translation.y:.3f} '
                f'z={t.transform.translation.z:.3f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfEchoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()