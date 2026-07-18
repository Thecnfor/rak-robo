from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory('competition_orchestrator_pkg'))
        / 'config'
        / 'orchestrator.yaml'
    )
    return LaunchDescription([
        Node(
            package='competition_orchestrator_pkg',
            executable='air_ground_orchestrator',
            name='air_ground_orchestrator',
            output='screen',
            parameters=[str(config)],
        )
    ])
