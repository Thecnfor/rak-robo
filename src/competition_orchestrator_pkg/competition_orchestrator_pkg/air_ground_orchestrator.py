"""Arena state aggregation plus public drone and cargo action adapters."""

import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from grasp_demo_interfaces.action import CargoDoorCommand, DroneFlightCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import Bool, String

from .cargo_contract import cargo_command_and_expected
from .mission_gate import door_action_allowed, flight_action_allowed, MissionGate


def _duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class AirGroundOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__('air_ground_orchestrator')
        self.declare_parameter('auto_dispatch_after_ground', True)
        self.declare_parameter('default_action_timeout', 30.0)
        self.declare_parameter('return_height', 1.8)
        self.declare_parameter('allow_manual_flight_actions', False)
        self.declare_parameter('allow_manual_door_actions', False)
        self._auto_dispatch = bool(self.get_parameter('auto_dispatch_after_ground').value)
        self._default_timeout = float(self.get_parameter('default_action_timeout').value)
        self._return_height = float(self.get_parameter('return_height').value)
        self._allow_manual_flight = bool(
            self.get_parameter('allow_manual_flight_actions').value
        )
        self._allow_manual_door = bool(
            self.get_parameter('allow_manual_door_actions').value
        )
        self._gate = MissionGate()
        self._ground_state = 'UNKNOWN'
        self._drone_state = 'IDLE'
        self._cargo_state = 'UNKNOWN'
        self._landed = False
        self._have_landed_status = False
        self._pose: Optional[PoseStamped] = None
        self._home: Optional[PoseStamped] = None

        transient = QoSProfile(depth=10)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._arena_state_pub = self.create_publisher(
            String, '/arena/orchestrator/state', transient
        )
        self._mission_pub = self.create_publisher(
            Bool, '/drone/navigation/mission_request', transient
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, '/drone/navigation/operator_goal', transient
        )
        self._mode_pub = self.create_publisher(
            String, '/drone/navigation/operator_mode', transient
        )
        self._cargo_pub = self.create_publisher(String, '/cargo_bay/command', 10)
        self.create_subscription(String, '/arena/ground/state', self._on_ground_state, transient)
        self.create_subscription(
            String, '/drone/navigation/state', self._on_drone_state, transient
        )
        self.create_subscription(String, '/cargo_bay/status', self._on_cargo_state, 10)
        self.create_subscription(Bool, '/drone/navigation/landed', self._on_landed, transient)
        self.create_subscription(Odometry, '/drone/navigation/odometry', self._on_odometry, 20)
        self.create_timer(0.2, self._publish_arena_state)

        self._flight_action = ActionServer(
            self,
            DroneFlightCommand,
            '/drone/flight_command',
            execute_callback=self._execute_flight,
            goal_callback=self._flight_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self._door_action = ActionServer(
            self,
            CargoDoorCommand,
            '/cargo_bay/door_command',
            execute_callback=self._execute_door,
            goal_callback=self._door_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def _on_ground_state(self, message: String) -> None:
        self._ground_state = message.data.strip().upper()
        if self._auto_dispatch and self._gate.observe(self._ground_state):
            self._publish_mode('CLEAR')
            request = Bool()
            request.data = True
            self._mission_pub.publish(request)

    def _on_drone_state(self, message: String) -> None:
        self._drone_state = message.data

    def _on_cargo_state(self, message: String) -> None:
        self._cargo_state = message.data

    def _on_landed(self, message: Bool) -> None:
        self._have_landed_status = True
        self._landed = message.data

    def _on_odometry(self, message: Odometry) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self._pose = pose
        if self._home is None and self._landed:
            self._home = pose

    async def _cooperative_sleep(self, seconds: float) -> None:
        """Yield to rclpy without depending on an external asyncio event loop."""
        future = Future()

        def wake() -> None:
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(seconds, wake)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def _publish_arena_state(self) -> None:
        message = String()
        message.data = (
            f'ground={self._ground_state} drone={self._drone_state} '
            f'cargo="{self._cargo_state}" landed={str(self._landed).lower()}'
        )
        self._arena_state_pub.publish(message)

    def _flight_goal(self, goal_request) -> GoalResponse:
        valid = {
            DroneFlightCommand.Goal.TAKEOFF,
            DroneFlightCommand.Goal.GOTO,
            DroneFlightCommand.Goal.RETURN,
            DroneFlightCommand.Goal.HOVER,
            DroneFlightCommand.Goal.LAND,
            DroneFlightCommand.Goal.ABORT,
        }
        valid_command = goal_request.command in valid
        allowed = flight_action_allowed(
            goal_request.command, self._drone_state, self._allow_manual_flight
        )
        return GoalResponse.ACCEPT if valid_command and allowed else GoalResponse.REJECT

    def _door_goal(self, goal_request) -> GoalResponse:
        valid_door = goal_request.door in {
            CargoDoorCommand.Goal.SIDE,
            CargoDoorCommand.Goal.BOTTOM,
        }
        valid_command = goal_request.command in {
            CargoDoorCommand.Goal.OPEN,
            CargoDoorCommand.Goal.CLOSE,
        }
        allowed = door_action_allowed(self._drone_state, self._allow_manual_door)
        return (
            GoalResponse.ACCEPT
            if valid_door and valid_command and allowed
            else GoalResponse.REJECT
        )

    def _publish_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self._mode_pub.publish(message)

    def _distance_to(self, target: PoseStamped) -> float:
        if self._pose is None:
            return math.inf
        dx = target.pose.position.x - self._pose.pose.position.x
        dy = target.pose.position.y - self._pose.pose.position.y
        dz = target.pose.position.z - self._pose.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    async def _execute_flight(self, goal_handle):
        goal = goal_handle.request
        result = DroneFlightCommand.Result()
        timeout = _duration_seconds(goal.timeout) or self._default_timeout
        tolerance = float(goal.position_tolerance) or 0.2
        target: Optional[PoseStamped] = None

        if goal.command == DroneFlightCommand.Goal.TAKEOFF:
            self._publish_mode('CLEAR')
            request = Bool()
            request.data = True
            self._mission_pub.publish(request)
        elif goal.command == DroneFlightCommand.Goal.GOTO:
            target = goal.target
            self._goal_pub.publish(target)
            self._publish_mode('TRAJECTORY')
        elif goal.command == DroneFlightCommand.Goal.RETURN:
            if self._home is None:
                result.reason = 'home pose has not been recorded'
                goal_handle.abort()
                return result
            target = PoseStamped()
            target.header = self._home.header
            target.pose = self._home.pose
            target.pose.position.z = self._return_height
            self._goal_pub.publish(target)
            self._publish_mode('RETURN')
        elif goal.command == DroneFlightCommand.Goal.HOVER:
            self._publish_mode('HOLD')
            result.success = True
            result.reason = 'hold requested'
            goal_handle.succeed()
            return self._finish_flight_result(result)
        elif goal.command in {DroneFlightCommand.Goal.LAND, DroneFlightCommand.Goal.ABORT}:
            self._publish_mode('LAND')

        started = time.monotonic()
        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                self._publish_mode('HOLD')
                result.reason = 'cancelled'
                goal_handle.canceled()
                return self._finish_flight_result(result)
            feedback = DroneFlightCommand.Feedback()
            feedback.state = self._drone_state
            feedback.remaining_distance = float(self._distance_to(target)) if target else math.inf
            goal_handle.publish_feedback(feedback)
            if target is not None:
                complete = self._distance_to(target) <= tolerance
            elif goal.command == DroneFlightCommand.Goal.TAKEOFF:
                complete = any(
                    phase in self._drone_state
                    for phase in (
                        'EGO_TRANSIT',
                        'TARGET_SEARCH',
                        'VISUAL_ALIGN',
                        'DROP_HOLD',
                        'RETURN',
                    )
                )
            else:
                complete = self._have_landed_status and self._landed
            if complete:
                if target is not None:
                    self._publish_mode('HOLD')
                result.success = True
                result.reason = 'command completed'
                goal_handle.succeed()
                return self._finish_flight_result(result)
            await self._cooperative_sleep(0.05)
        result.reason = 'command timeout'
        self._publish_mode('LAND')
        goal_handle.abort()
        return self._finish_flight_result(result)

    def _finish_flight_result(self, result):
        if self._pose is not None:
            result.final_pose = self._pose
        return result

    async def _execute_door(self, goal_handle):
        goal = goal_handle.request
        result = CargoDoorCommand.Result()
        timeout = _duration_seconds(goal.timeout) or 5.0
        door = 'left' if goal.door == CargoDoorCommand.Goal.SIDE else 'bottom'
        action = 'open' if goal.command == CargoDoorCommand.Goal.OPEN else 'close'
        command, expected = cargo_command_and_expected(door, action)
        started = time.monotonic()
        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                result.reason = 'cancelled'
                goal_handle.canceled()
                return result
            message = String()
            message.data = command
            self._cargo_pub.publish(message)
            feedback = CargoDoorCommand.Feedback()
            feedback.current_state = self._cargo_state
            goal_handle.publish_feedback(feedback)
            if expected in self._cargo_state:
                result.success = True
                result.final_state = self._cargo_state
                result.reason = 'door command confirmed'
                goal_handle.succeed()
                return result
            await self._cooperative_sleep(0.1)
        result.final_state = self._cargo_state
        result.reason = 'door command timeout'
        goal_handle.abort()
        return result

    def destroy_node(self):
        self._flight_action.destroy()
        self._door_action.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirGroundOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
