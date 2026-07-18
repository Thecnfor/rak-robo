#!/usr/bin/env python3
"""把 host 端 /cmd_vel 转发给 Isaac Sim 的 differential controller。

用法（在 isaac 5.1 端 Kit GUI 里）：
  1. 建一个场景 + 带 differential drive 的机器人
  2. 加 ROS2 OmniGraph: ROS2Context + ROS2SubscribeTwist → DifferentialController
  3. 启动这个 node，host 端 ros2 topic pub /cmd_vel ... 就能动

host 端跑：
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0.5},angular:{z:0.3}}" -r 10

设计目的：本节点其实**只是占位 demo**，Isaac 端 OmniGraph 自己也会订阅 /cmd_vel，
所以本节点默认行为是把收到 Twist 打 log 一下即可。把它当成"链路验证桩"。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.get_logger().info(
            f'cmd_vel: linear.x={msg.linear.x:.2f} '
            f'angular.z={msg.angular.z:.2f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()