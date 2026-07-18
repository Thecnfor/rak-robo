#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from grasp_demo_interfaces.action import DetectObject, GripperCommand, PlanToPose


class PickPlaceStateMachine(Node):
    def __init__(self):
        super().__init__('pick_place_state_machine')
        self.declare_parameter('command_topic', '/hand_command')
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('observation_pose', [0.0, 0.58, -1.67, -0.5, 1.51, 0.0])
        self.declare_parameter('target_classes', ['pencil', 'pen'])
        self.declare_parameter('confidence_threshold', 0.01)
        self.declare_parameter('object_point_topic', '/demo_grasp/object_point_base')
        self.declare_parameter('normal_topic', '/demo_grasp/normal_base')
        self.declare_parameter('long_axis_topic', '/demo_grasp/long_axis_base')
        self.declare_parameter('pre_grasp_offset', 0.08)
        self.declare_parameter('finger_tip_offset', 0.03)
        self.declare_parameter('lift_offset', 0.12)
        self.declare_parameter('place_position', [0.65, -0.22, 0.18])

        self.object_point = None
        self.normal = None
        self.long_axis = None
        self.hand_pub = self.create_publisher(JointState, self.get_parameter('command_topic').value, 10)
        self.create_subscription(PointStamped, self.get_parameter('object_point_topic').value, self.on_point, 10)
        self.create_subscription(Vector3Stamped, self.get_parameter('normal_topic').value, self.on_normal, 10)
        self.create_subscription(Vector3Stamped, self.get_parameter('long_axis_topic').value, self.on_axis, 10)
        self.detect_client = ActionClient(self, DetectObject, '/demo_detect_object')
        self.plan_client = ActionClient(self, PlanToPose, '/demo_plan_to_pose')
        self.gripper_client = ActionClient(self, GripperCommand, '/demo_gripper_command')
        self.get_logger().info('perception_pipeline_state_machine started; this demo computes candidate poses but does not execute grasping')

    def on_point(self, msg):
        self.object_point = msg

    def on_normal(self, msg):
        self.normal = msg

    def on_axis(self, msg):
        self.long_axis = msg

    @staticmethod
    def _fmt_point(msg):
        return f'({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) frame={msg.header.frame_id}'

    @staticmethod
    def _fmt_vector(msg):
        return f'({msg.vector.x:.3f}, {msg.vector.y:.3f}, {msg.vector.z:.3f}) frame={msg.header.frame_id}'

    def move_to_observation_pose(self, hold_seconds=3.0):
        msg = JointState()
        msg.name = list(self.get_parameter('joint_names').value)
        msg.position = [float(v) for v in self.get_parameter('observation_pose').value]
        end_time = time.time() + hold_seconds
        while rclpy.ok() and time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.hand_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)
        self.get_logger().info('observation pose command sent')

    def detect_object(self):
        if not self.detect_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/demo_detect_object unavailable')
        goal = DetectObject.Goal()
        goal.target_classes = [str(v) for v in self.get_parameter('target_classes').value]
        goal.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        goal.timeout = 15.0
        future = self.detect_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('detect goal rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'detect failed: {result.message}')
        self.get_logger().info(f'detected {result.detected_class}, confidence={result.confidence:.3f}')

    def _wait_for_inputs(self, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.object_point is not None and self.normal is not None and self.long_axis is not None:
                return True
        return False

    def _normalized_normal(self):
        if self.normal is None:
            return (0.0, 0.0, -1.0)
        nx, ny, nz = self.normal.vector.x, self.normal.vector.y, self.normal.vector.z
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm < 1e-9:
            return (0.0, 0.0, -1.0)
        return (nx / norm, ny / norm, nz / norm)

    def _offset_along_approach(self, source, distance):
        nx, ny, nz = self._normalized_normal()
        msg = PointStamped()
        msg.header = source.header
        msg.point.x = source.point.x - nx * distance
        msg.point.y = source.point.y - ny * distance
        msg.point.z = source.point.z - nz * distance
        return msg

    def _point_from_xyz(self, xyz):
        msg = PointStamped()
        msg.header = self.object_point.header
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        return msg

    def _send_plan(self, point, label, execute=True):
        if not self.plan_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/demo_plan_to_pose unavailable')
        goal = PlanToPose.Goal()
        goal.target = point
        goal.normal = self.normal
        goal.long_axis = self.long_axis
        goal.execute = execute
        goal.label = label
        future = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'plan goal rejected: {label}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'plan failed: {label}: {result.message}')
        self.get_logger().info(f'{label}: {result.message}')

    def _send_gripper(self, command, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/demo_gripper_command unavailable')
        goal = GripperCommand.Goal()
        goal.command = command
        goal.position = float(position)
        goal.speed = 50.0
        goal.wait_for_completion = True
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'gripper goal rejected: {command}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'gripper failed: {result.message}')
        self.get_logger().info(f'gripper {command}: final_position={result.final_position:.1f}')

    def run_once(self):
        try:
            self.move_to_observation_pose()
            self.detect_object()
            if not self._wait_for_inputs():
                raise RuntimeError('perception outputs timed out')
            self.get_logger().info(f'object_point_base={self._fmt_point(self.object_point)}')
            self.get_logger().info(f'normal_base={self._fmt_vector(self.normal)}')
            self.get_logger().info(f'long_axis_base={self._fmt_vector(self.long_axis)}')
            tip = float(self.get_parameter('finger_tip_offset').value)
            pre = float(self.get_parameter('pre_grasp_offset').value)
            lift = float(self.get_parameter('lift_offset').value)
            pre_grasp = self._offset_along_approach(self.object_point, tip + pre)
            grasp = self._offset_along_approach(self.object_point, tip)
            lifted = self._offset_along_approach(self.object_point, tip + lift)
            place = self._point_from_xyz(self.get_parameter('place_position').value)
            pre_place = self._offset_along_approach(place, pre)
            self.get_logger().info(f'pre_grasp={self._fmt_point(pre_grasp)}')
            self.get_logger().info(f'grasp={self._fmt_point(grasp)}')
            self.get_logger().info(f'lifted={self._fmt_point(lifted)}')
            self.get_logger().info(f'place={self._fmt_point(place)}')
            self._send_plan(pre_grasp, 'candidate_pre_grasp', execute=False)
            self._send_plan(grasp, 'candidate_grasp', execute=False)
            self._send_plan(lifted, 'candidate_lift', execute=False)
            self._send_plan(pre_place, 'candidate_pre_place', execute=False)
            self._send_plan(place, 'candidate_place', execute=False)
            self.get_logger().info('candidate grasp/place poses generated; real IK, trajectory planning, and execution are left for the competition task')
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return False
        self.get_logger().info('perception pipeline demo completed')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceStateMachine()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
