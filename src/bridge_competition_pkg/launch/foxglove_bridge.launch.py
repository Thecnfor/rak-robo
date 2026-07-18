"""
Launch the always-on Foxglove observation bridge.

**端口 8765 固定，address 0.0.0.0 固定**——给所有 host 上跑的 ROS 2 节点做常驻观测。
不要为了"避开冲突"改端口，冲突了就 kill 旧实例。

仅暴露性能相关参数：
- use_compression: WebSocket 压缩
- num_threads:    处理线程数

参数列表（topic 白名单 / capabilities / QoS 覆盖）写在
config/foxglove_bridge.yaml，需要时改 YAML 而不是改 launch。

单独跑:
    ros2 launch bridge_competition_pkg foxglove_bridge.launch.py

被 host_bridge_bringup.launch.py 默认包含，常驻运行。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = (
        Path(get_package_share_directory('bridge_competition_pkg'))
        / 'config'
        / 'foxglove_bridge.yaml'
    )
    declared = [
        DeclareLaunchArgument(
            'use_compression', default_value='true',
            description='WebSocket 压缩；高频/图像 topic 强烈建议开',
        ),
        DeclareLaunchArgument(
            'num_threads', default_value='4',
            description='处理线程数；0 = 单线程',
        ),
    ]

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        # 端口 + bind 写死，不暴露
        parameters=[
            str(config),
            {
                'port': 8765,
                'address': '0.0.0.0',
                'use_compression': LaunchConfiguration('use_compression'),
                'num_threads': LaunchConfiguration('num_threads'),
            },
        ],
    )

    return LaunchDescription(declared + [foxglove_node])
