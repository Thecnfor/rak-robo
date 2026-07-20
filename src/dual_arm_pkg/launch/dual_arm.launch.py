"""Bring up the Mercury X1 dual-arm control stack.

This launch is a thin wrapper around the three competition dual-arm nodes
defined in M5.1 (`dual_arm_observation_node`, `dual_gripper_server_node`,
`dual_arm_pick_place_node`). The host_bridge_bringup launch in
`bridge_competition_pkg` includes this file so the ground-side M3 and the
air-side M4 can share one launch command.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='dual_arm_pkg',
            executable='dual_arm_observation_node',
            name='dual_arm_observation_node',
            output='screen',
        ),
        Node(
            package='dual_arm_pkg',
            executable='dual_gripper_server_node',
            name='dual_gripper_server_node',
            output='screen',
        ),
        Node(
            package='dual_arm_pkg',
            executable='dual_arm_pick_place_node',
            name='dual_arm_pick_place_node',
            output='screen',
        ),
    ])
