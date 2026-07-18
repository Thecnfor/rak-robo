#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ObservationPoseNode(Node):
    def __init__(self):
        super().__init__('observation_pose_node')
        self.declare_parameter('command_topic', '/hand_command')
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('observation_pose', [0.0, 0.58, -1.67, -0.5, 1.51, 0.0])
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('hold_seconds', 3.0)
        self.pub = self.create_publisher(JointState, self.get_parameter('command_topic').value, 10)
        self.get_logger().info('observation_pose_node started')

    def publish_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.get_parameter('joint_names').value)
        msg.position = [float(v) for v in self.get_parameter('observation_pose').value]
        self.pub.publish(msg)

    def run(self):
        rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        hold_seconds = max(0.1, float(self.get_parameter('hold_seconds').value))
        period = 1.0 / rate_hz
        end_time = time.time() + hold_seconds
        while rclpy.ok() and time.time() < end_time:
            self.publish_once()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.get_logger().info('observation pose command published')


def main(args=None):
    rclpy.init(args=args)
    node = ObservationPoseNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
