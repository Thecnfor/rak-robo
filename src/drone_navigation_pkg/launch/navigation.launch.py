"""Launch the single PX4 control chain and EGO-inspired local planning stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory('drone_navigation_pkg'))
        / 'config'
        / 'navigation.yaml'
    )
    fixed_diagnostic = LaunchConfiguration('allow_fixed_setpoint_diagnostic')
    common = {
        'parameters': [
            str(config),
            {'allow_fixed_setpoint_diagnostic': fixed_diagnostic},
        ],
        'output': 'screen',
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_fixed_setpoint_diagnostic',
            default_value='false',
        ),
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
