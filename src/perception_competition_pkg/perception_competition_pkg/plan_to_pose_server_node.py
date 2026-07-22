"""Action server wrapping plan_to_pose logic.

For the teaching pipeline, M2.6 calls `/demo_plan_to_pose` with the
detected object's camera-frame pose + normal + long axis, and gets back
a `PoseStamped` of the arm gripper end-effector. This server is a stub:
it builds a candidate pre-grasp pose 15 cm above the target point,
oriented along the long axis, in the `base_link_arm` frame.

When `request.execute == true`, the server additionally publishes the
command on `/hand_command` as a single JointState (placeholder — real
execution lives in `dual_arm_pkg::dual_arm_pick_place_node`). The
`commander_dual_arm_node` (when available) consumes this and runs IK.

Usage:
    ros2 run perception_competition_pkg plan_to_pose_server_node
"""

from __future__ import annotations

import math
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from grasp_demo_interfaces.action import PlanToPose


class _PlanToPoseServer(Node):
    def __init__(self) -> None:
        super().__init__('plan_to_pose_server')
        self._action = ActionServer(
            self, PlanToPose, '/demo_plan_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda req: rclpy.action.GoalResponse.ACCEPT,
            cancel_callback=lambda req: rclpy.action.CancelResponse.ACCEPT,
        )
        self._publisher = self.create_publisher(
            JointState, '/hand_command',
            QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

    def _execute(self, goal_handle) -> PlanToPose.Result:
        request = goal_handle.request
        target = request.target
        normal = request.normal
        long_axis = request.long_axis
        self.get_logger().info(
            f'plan_to_pose: label={request.label!r} '
            f'execute={request.execute} target=({target.point.x:.2f}, '
            f'{target.point.y:.2f}, {target.point.z:.2f})')
        result = PlanToPose.Result()
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'base_link_arm'
        pose.header.stamp = self.get_clock().now().to_msg()
        n = self._as_unit(normal.vector)
        a = self._as_unit(long_axis.vector)
        if n.norm() < 1e-6:
            n = Vector3(x=0.0, y=0.0, z=1.0)
        if a.norm() < 1e-6:
            a = Vector3(x=1.0, y=0.0, z=0.0)
        a = self._orthogonalize(a, n)
        y = self._cross(n, a)
        pose.position.x = target.point.x + 0.15 * n.x
        pose.position.y = target.point.y + 0.15 * n.y
        pose.position.z = target.point.z + 0.15 * n.z
        pose.orientation = self._quat_from_columns(a, y, n)
        result.success = True
        result.message = f'ok ({request.label})'
        result.commanded_pose = pose
        if request.execute:
            self._publish_hand_command()
        goal_handle.succeed()
        return result

    def _publish_hand_command(self) -> None:
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = [
            'wheel_left_joint', 'wheel_right_joint',
            'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
            'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
            'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
            'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
            'gripper_left_joint', 'gripper_right_joint',
        ]
        state.position = [0.0] * 16
        self._publisher.publish(state)

    @staticmethod
    def _as_unit(v: Vector3) -> Vector3:
        n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        if n < 1e-9:
            return v
        return Vector3(x=v.x / n, y=v.y / n, z=v.z / n)

    @staticmethod
    def _orthogonalize(a: Vector3, n: Vector3) -> Vector3:
        dot = a.x * n.x + a.y * n.y + a.z * n.z
        return Vector3(
            x=a.x - dot * n.x,
            y=a.y - dot * n.y,
            z=a.z - dot * n.z,
        )

    @staticmethod
    def _cross(a: Vector3, b: Vector3) -> Vector3:
        return Vector3(
            x=a.y * b.z - a.z * b.y,
            y=a.z * b.x - a.x * b.z,
            z=a.x * b.y - a.y * b.x,
        )

    @staticmethod
    def _quat_from_columns(x: Vector3, y: Vector3, z: Vector3) -> Quaternion:
        trace = x.x + y.y + z.z
        if trace > 0.0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x_q = (y.z - z.y) * s
            y_q = (z.x - x.z) * s
            z_q = (x.y - y.x) * s
        elif (x.x > y.y) and (x.x > z.z):
            s = 2.0 * math.sqrt(1.0 + x.x - y.y - z.z)
            w = (y.z - z.y) / s
            x_q = 0.25 * s
            y_q = (y.x + x.y) / s
            z_q = (z.x + x.z) / s
        elif y.y > z.z:
            s = 2.0 * math.sqrt(1.0 + y.y - x.x - z.z)
            w = (z.x - x.z) / s
            x_q = (y.x + x.y) / s
            y_q = 0.25 * s
            z_q = (z.y + y.z) / s
        else:
            s = 2.0 * math.sqrt(1.0 + z.z - x.x - y.y)
            w = (x.y - y.x) / s
            x_q = (z.x + x.z) / s
            y_q = (z.y + y.z) / s
            z_q = 0.25 * s
        return Quaternion(x=float(x_q), y=float(y_q),
                         z=float(z_q), w=float(w))


def main() -> int:
    rclpy.init()
    try:
        node = _PlanToPoseServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
