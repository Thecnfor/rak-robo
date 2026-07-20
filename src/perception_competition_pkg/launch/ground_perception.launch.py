"""Bring up the ground-side perception stack (YOLOE + depth pose).

Wires `yoloe_detector_node` and `depth_pose_estimator_node` so the grasp
pipeline sees a steady `/demo_grasp/*` topic set even when the real
YOLOE weights are not yet deployed. Both nodes run in stub mode by
default and produce synthetic 200x200 detections at 0.5 s intervals.

To enable real YOLOE inference, place `weights/yoloe-26l-seg.pt` in
`src/perception_competition_pkg/weights/` (gitignored) and replace the
body of `YoloeDetector._publish_synthetic` in `yoloe_detector_node.py`.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='perception_competition_pkg',
            executable='yoloe_detector_node',
            name='yoloe_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/arm_camera/rgb',
                'simulate_target_label': 'pencil',
                'simulate_target_confidence': 0.92,
            }],
        ),
        Node(
            package='perception_competition_pkg',
            executable='depth_pose_estimator_node',
            name='depth_pose_estimator_node',
            output='screen',
            parameters=[{
                'depth_topic': '/arm_camera/depth',
                'camera_info_topic': '/arm_camera/camera_info',
                'bbox_topic': '/demo_grasp/bbox',
                'simulated_distance': 1.2,
            }],
        ),
        Node(
            package='perception_competition_pkg',
            executable='detect_object_server_node',
            name='detect_object_server_node',
            output='screen',
        ),
        Node(
            package='perception_competition_pkg',
            executable='plan_to_pose_server_node',
            name='plan_to_pose_server_node',
            output='screen',
        ),
    ])
