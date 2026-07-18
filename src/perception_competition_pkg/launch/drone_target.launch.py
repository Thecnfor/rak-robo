from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory('perception_competition_pkg'))
        / 'config'
        / 'drone_target.yaml'
    )
    return LaunchDescription([
        Node(
            package='perception_competition_pkg',
            executable='drone_target_detector',
            name='drone_target_detector',
            output='screen',
            parameters=[str(config)],
        )
    ])
