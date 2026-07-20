"""
One-command host bringup for the competition UAV chain.

Foxglove 始终常驻（端口 8765 固定）—— 调试时不必再开 / 关。
如需停止观测就停整个 bringup，不要尝试单独关 Foxglove。

用法:
    ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py
    ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py record_bag:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(package: str, filename: str):
    return PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / 'launch' / filename)
    )


def generate_launch_description() -> LaunchDescription:
    record_bag = LaunchConfiguration('record_bag')
    report_path = LaunchConfiguration('interface_report_path')
    return LaunchDescription([
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument(
            'interface_report_path',
            default_value='/tmp/drone_interface_report.json',
        ),
        # 业务节点
        IncludeLaunchDescription(_launch('drone_navigation_pkg', 'navigation.launch.py')),
        IncludeLaunchDescription(_launch('perception_competition_pkg', 'drone_target.launch.py')),
        IncludeLaunchDescription(_launch('perception_competition_pkg', 'ground_perception.launch.py')),
        IncludeLaunchDescription(
            _launch('competition_orchestrator_pkg', 'orchestrator.launch.py')
        ),
        IncludeLaunchDescription(_launch('dual_arm_pkg', 'dual_arm.launch.py')),
        # 独立常驻运行，空白名单让任何业务节点起的 topic 自动可见；改 YAML 后用
        #   systemctl --user restart foxglove-bridge.service
        # 即可重启，不用动这一坨。详见 docs/setup/foxglove_setup.md
        # 接口健康检查
        Node(
            package='bridge_competition_pkg',
            executable='drone_interface_audit',
            name='drone_interface_audit',
            output='screen',
            parameters=[{'use_sim_time': True, 'report_path': report_path}],
        ),
        # rosbag 可选
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', '/tmp/drone_competition_bag',
                '/drone/navigation/odometry',
                '/drone/navigation/state',
                '/drone/navigation/planned_path',
                '/drone/drop_target_offset',
                '/avoidance/lidar/pointcloud',
                '/fmu/out/vehicle_status_v1',
                '/cargo_bay/status',
            ],
            output='screen',
            condition=IfCondition(record_bag),
        ),
    ])
