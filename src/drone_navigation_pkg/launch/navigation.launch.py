"""Launch the single PX4 control chain and EGO-inspired local planning stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory('drone_navigation_pkg'))
        / 'config'
        / 'navigation.yaml'
    )
    common = {'parameters': [str(config)], 'output': 'screen'}
    return LaunchDescription([
        Node(
            package='drone_navigation_pkg',
            executable='px4_state_adapter',
            name='px4_state_adapter',
            **common,
        ),
        Node(
            package='drone_navigation_pkg',
            executable='ego_local_planner',
            name='ego_local_planner',
            **common,
        ),
        Node(
            package='drone_navigation_pkg',
            executable='trajectory_executor',
            name='trajectory_executor',
            **common,
        ),
        Node(
            package='drone_navigation_pkg',
            executable='flight_supervisor',
            name='flight_supervisor',
            **common,
        ),
    ])
