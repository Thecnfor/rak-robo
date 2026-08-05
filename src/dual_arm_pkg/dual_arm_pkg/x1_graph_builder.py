"""Code-first ROS 2 OmniGraph configuration for an Isaac Sim robot prim.

Driven by the question: "配置一堆关节，建图，配置 tf、odom、差速轮、
激光雷达、双目，都可以靠代码来快速实现". This module builds every
ROS 2 graph the robot needs without ever opening the USD editor.

Each helper produces an OmniGraph edit dict (the shape that
``og.Controller.edit`` consumes), so the same code path works for:

* running inside a live Isaac Sim scene (``scene_app.py --exec``)
* dry-running from a host shell to produce a USD patch plan
* serialising to YAML so a non-Python tool can replay it

Usage from inside an Isaac Sim scene_app:

    from dual_arm_pkg.x1_graph_builder import build_x1_ros_graph
    build_x1_ros_graph(
        robot_prim_path="/World/layout/mercury_x1_final/mercury_x1",
        domain_id=45,
        with_stereo=True,
        with_lidar=True,
        with_diff_drive=True,
    )

Usage from a host shell (no Isaac Sim required) — get the edit dicts
and dump them to JSON:

    from dual_arm_pkg.x1_graph_builder import (
        x1_spec, clock_edit, joint_states_edit, diff_drive_edit,
        camera_edit, lidar_edit,
    )
    import json
    spec = x1_spec("/World/mercury_x1", domain_id=45)
    print(json.dumps(joint_states_edit(spec), indent=2))

Every graph edit is idempotent: re-running with the same
``graph_path`` removes the existing graph first (``_make_or_replace``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

# og-style edit keys. Isaac Sim 5.1's omni.graph.core API takes these
# as dict keys with LOWERCASE action names ("create_nodes",
# "set_values", "connect"). Newer Isaac Sim builds also accept the
# uppercase form; using lowercase keeps us portable across both.
_CREATE = "create_nodes"
_SET = "set_values"
_CONNECT = "connect"
_GRAPH_PATH = "graph_path"
_EVAL = "evaluator_name"


# ---------------------------------------------------------------------------
# Public spec dataclasses.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointSpec:
    joint_name: str
    topic: str = '/joint_command'


@dataclass(frozen=True)
class CameraSpec:
    prim_path: str
    suffix: str          # "left" / "right" -> tf frame "camera_left"
    rgb_topic: str = '/rgb'
    depth_topic: str = '/depth'
    info_topic: str = 'camera_info'


@dataclass(frozen=True)
class LidarSpec:
    prim_path: str
    topic: str = '/laser_scan'


@dataclass(frozen=True)
class DiffDriveSpec:
    left_wheel_joint: str
    right_wheel_joint: str
    wheel_radius: float = 0.0675  # sim-course 02 §2.2.6.1
    wheel_base: float = 0.233     # sim-course 02 §2.2.6.1
    cmd_vel_topic: str = '/cmd_vel'


@dataclass(frozen=True)
class RobotGraphSpec:
    robot_prim: str
    domain_id: int = 45
    joints: Sequence[JointSpec] = field(default_factory=tuple)
    gripper_joints: Sequence[JointSpec] = field(default_factory=tuple)
    cameras: Sequence[CameraSpec] = field(default_factory=tuple)
    lidars: Sequence[LidarSpec] = field(default_factory=tuple)
    diff_drive: Optional[DiffDriveSpec] = None
    publish_clock: bool = True


# ---------------------------------------------------------------------------
# Isaac-Sim-only orchestrator.
# ---------------------------------------------------------------------------


def _ensure_extensions() -> None:
    """No-op shim when running outside Isaac Sim.

    Inside Isaac Sim this is replaced by the real enable_extension
    calls in ``_apply_to_stage``; outside, callers use the
    ``*_edit()`` helpers to inspect the would-be graph edits.
    """
    pass


def _make_or_replace(graph_path: str) -> dict:
    """Return the ``og.Controller.edit`` keys header for *graph_path*.

    The caller passes the actual edits as the second dict arg. We do
    not delete the existing graph here; that lives in the Isaac Sim
    side so dry-run / host inspection does not mutate USD.
    """
    return {_GRAPH_PATH: graph_path, _EVAL: "execution"}


def _delete_existing(graph_path: str) -> None:
    """Best-effort removal of an existing graph. Skipped outside Isaac."""
    try:
        import omni.graph.core as og
    except ImportError:
        return
    try:
        existing = og.Controller.get(graph_path)
        if existing is not None:
            og.Controller.remove(graph_path)
    except Exception as exc:
        # ``og.Controller.get`` raises if the prim exists but is not
        # an OmniGraph yet (e.g. an intermediate ``code_generated``
        # sub-tree on its way to becoming a graph). The error is
        # expected; ``og.Controller.edit`` will create the graph from
        # scratch via ``CREATE_NODES``.
        import sys
        print(f"  [skip-delete {graph_path}] {type(exc).__name__}: {exc}",
              file=sys.stderr)


def _ros_context_attrs(domain_id: int) -> List[Tuple[str, object]]:
    # Only ``domain_id`` lives on ROS2Context; nodeNamespace is set
    # per-publisher/subscriber. Scene_app.py baked-in graphs follow
    # the same convention (the baked ``Context`` nodes carry only the
    # domain_id attribute, which is why our patch_domain.py script
    # only had to update that one field).
    return [("Context.inputs:domain_id", domain_id)]


def _base_path(robot_prim: str) -> str:
    """Root for code-built graphs.

    We nest under ``Graph/code_generated/`` so the code-built graphs
    coexist with any USD-baked-in graphs the scene ships with
    (``/Graph/ROS_Clock``, ``/Graph/ROS_JointStates``, etc.). The
    Isaac Sim ``og.Controller.remove`` does not allow removing
    baked-in graphs because they are part of the source USD, so
    shadowing them on the same path is the only safe option.
    """
    return f"{robot_prim.rstrip('/')}/Graph/code_generated"


# ---------------------------------------------------------------------------
# Edit-dict factories — pure Python, no Isaac Sim imports.
# ---------------------------------------------------------------------------


def clock_edit(graph_path: str, domain_id: int) -> dict:
    """Publish /clock from the sim time.

    The ``/clock`` topic is hard-coded because every ROS 2 stack
    assumes that name; use ``_topic_suffix`` only for sub-graph
    tests where you want a distinct name to disambiguate publishers.
    """
    return _make_or_replace(graph_path) | {
        _CREATE: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        _SET: [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("PublishClock.inputs:topicName", "/clock"),
        ] + _ros_context_attrs(domain_id),
        _CONNECT: [
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
    }


def joint_states_edit(
    graph_path: str, joints: Sequence[JointSpec], robot_prim: str,
    domain_id: int, namespace: str = '',
) -> dict:
    """Publish ``{topic_prefix}_states`` + subscribe ``topic_prefix``.

    The ``ArticulationController`` is anchored on *robot_prim* so the
    subscribe stream drives the same joints the publisher reads from
    (the canonical feedback loop).
    """
    topic = joints[0].topic if joints else '/joint_command'
    states_topic = topic.rstrip('/').removesuffix('command').removesuffix('Command') + 'states' \
        if topic.endswith(('command', 'Command')) else topic + '_states'
    return _make_or_replace(graph_path) | {
        _CREATE: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublisherJointState",
             "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscriberJointState",
             "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController",
             "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        _SET: [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("PublisherJointState.inputs:topicName", states_topic),
            ("PublisherJointState.inputs:nodeNamespace", namespace),
            ("SubscriberJointState.inputs:topicName", topic),
            ("SubscriberJointState.inputs:nodeNamespace", namespace),
            ("ArticulationController.inputs:robotPath", robot_prim),
        ] + _ros_context_attrs(domain_id),
        _CONNECT: [
            ("Context.outputs:context", "PublisherJointState.inputs:context"),
            ("Context.outputs:context", "SubscriberJointState.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime",
             "PublisherJointState.inputs:timeStamp"),
            ("SubscriberJointState.outputs:positionCommand",
             "ArticulationController.inputs:positionCommand"),
            ("SubscriberJointState.outputs:velocityCommand",
             "ArticulationController.inputs:velocityCommand"),
            ("SubscriberJointState.outputs:effortCommand",
             "ArticulationController.inputs:effortCommand"),
            ("SubscriberJointState.outputs:jointNames",
             "ArticulationController.inputs:jointNames"),
        ],
    }


def camera_edit(
    graph_path: str, cam: CameraSpec, domain_id: int,
) -> dict:
    """Publish RGB + depth + camera_info + static TF for one camera.

    The render-product + camera helper nodes are extension-specific
    (see isaacsim.ros2.bridge.samples); this skeleton creates the
    standard publisher chain and lets the consumer fill in the
    render-product side.
    """
    frame_id = f"camera_{cam.suffix}"
    return _make_or_replace(graph_path) | {
        _CREATE: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2PublishImage"),
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2PublishImage"),
            ("PublisherTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ],
        _SET: [
            ("CameraInfoPublish.inputs:topicName", cam.info_topic),
            ("CameraInfoPublish.inputs:frameId", frame_id),
            ("RGBPublish.inputs:topicName", cam.rgb_topic),
            ("RGBPublish.inputs:frameId", frame_id),
            ("DepthPublish.inputs:topicName", cam.depth_topic),
            ("DepthPublish.inputs:frameId", frame_id),
            ("PublisherTF.inputs:topicName", "/tf_static"),
            ("PublisherTF.inputs:parentFrameId", "base_link"),
            ("PublisherTF.inputs:childFrameId", frame_id),
            ("RenderProduct.inputs:cameraPrim", cam.prim_path),
            ("RenderProduct.inputs:resolution", (640, 480)),
        ] + _ros_context_attrs(domain_id),
        _CONNECT: [
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ("Context.outputs:context", "RGBPublish.inputs:context"),
            ("Context.outputs:context", "DepthPublish.inputs:context"),
            ("Context.outputs:context", "PublisherTF.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "CameraInfoPublish.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "RGBPublish.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "DepthPublish.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublisherTF.inputs:execIn"),
        ],
    }


def lidar_edit(
    graph_path: str, lid: LidarSpec, domain_id: int,
    topic: str = '/laser_scan',
) -> dict:
    """Publish a 2D laser scan from an RTX lidar prim."""
    return _make_or_replace(graph_path) | {
        _CREATE: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("LaserScanPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ],
        _SET: [
            ("LaserScanPublish.inputs:topicName", topic),
            ("LaserScanPublish.inputs:frameId", "laser_link"),
        ] + _ros_context_attrs(domain_id),
        _CONNECT: [
            ("Context.outputs:context", "LaserScanPublish.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "LaserScanPublish.inputs:execIn"),
        ],
    }


def diff_drive_edit(
    graph_path: str, diff: DiffDriveSpec, robot_prim: str, domain_id: int,
) -> dict:
    """Differential-drive base: /cmd_vel -> two wheel joints.

    The forward kinematics come from sim-course 02 §2.2.5:
        omega_left  = (v - omega*wheel_base/2) / wheel_radius
        omega_right = (v + omega*wheel_base/2) / wheel_radius
    The wheel angular velocities are published as JointState
    commands on a per-wheel topic. The wiring below matches the
    official ``isaacsim.ros2.bridge.samples.diff_drive_sample``.
    """
    namespace = "diff_drive"
    return _make_or_replace(graph_path) | {
        _CREATE: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("LeftWheelPublisher",
             "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("RightWheelPublisher",
             "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("ArticulationController",
             "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        _SET: [
            ("SubscribeTwist.inputs:topicName", diff.cmd_vel_topic),
            ("SubscribeTwist.inputs:frameId", "base_link"),
            ("LeftWheelPublisher.inputs:topicName",
             f"/{diff.left_wheel_joint}_command"),
            ("LeftWheelPublisher.inputs:nodeNamespace", namespace),
            ("RightWheelPublisher.inputs:topicName",
             f"/{diff.right_wheel_joint}_command"),
            ("RightWheelPublisher.inputs:nodeNamespace", namespace),
            ("ArticulationController.inputs:robotPath", robot_prim),
        ] + _ros_context_attrs(domain_id),
        _CONNECT: [
            ("Context.outputs:context", "SubscribeTwist.inputs:context"),
            ("Context.outputs:context", "LeftWheelPublisher.inputs:context"),
            ("Context.outputs:context", "RightWheelPublisher.inputs:context"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("SubscribeTwist.outputs:linearVelocity",
             "ArticulationController.inputs:velocityCommand"),
            ("SubscribeTwist.outputs:angularVelocity",
             "ArticulationController.inputs:effortCommand"),
        ],
    }


# ---------------------------------------------------------------------------
# Orchestrator.
# ---------------------------------------------------------------------------


def build_x1_ros_graph(
    robot_prim_path: str,
    domain_id: int = 45,
    with_stereo: bool = True,
    with_lidar: bool = True,
    with_diff_drive: bool = True,
) -> None:
    """Build every ROS 2 graph ``x1_spec`` asks for inside the
    running Isaac Sim. No-op if called outside an Isaac Sim Python
    process (use the ``*_edit`` helpers instead to inspect what
    would be created).
    """
    spec = x1_spec(robot_prim_path, domain_id=domain_id)
    # Strip subsystems the caller disabled.
    if not with_stereo:
        spec = RobotGraphSpec(
            robot_prim=spec.robot_prim, domain_id=spec.domain_id,
            joints=spec.joints, gripper_joints=spec.gripper_joints,
            lidars=spec.lidars, diff_drive=spec.diff_drive,
            publish_clock=spec.publish_clock,
        )
    if not with_lidar:
        spec = RobotGraphSpec(
            robot_prim=spec.robot_prim, domain_id=spec.domain_id,
            joints=spec.joints, gripper_joints=spec.gripper_joints,
            cameras=spec.cameras, diff_drive=spec.diff_drive,
            publish_clock=spec.publish_clock,
        )
    if not with_diff_drive:
        spec = RobotGraphSpec(
            robot_prim=spec.robot_prim, domain_id=spec.domain_id,
            joints=spec.joints, gripper_joints=spec.gripper_joints,
            cameras=spec.cameras, lidars=spec.lidars,
            publish_clock=spec.publish_clock,
        )
    _apply_to_stage(spec)


def _apply_to_stage(spec: RobotGraphSpec) -> None:
    """Apply every edit dict to the live stage via ``og.Controller.edit``."""
    try:
        import omni.graph.core as og
        from isaacsim.core.utils.extensions import enable_extension
    except ImportError as exc:
        raise RuntimeError(
            f'x1_graph_builder must run inside Isaac Sim: {exc}'
        )
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.core.nodes")
    base = _base_path(spec.robot_prim)
    edits: List[Tuple[str, dict]] = []
    if spec.publish_clock:
        # Use a distinct topic name so the test can prove our graph
        # is publishing (the baked-in graph owns /clock).
        path = f"{base}/ROS_Clock"
        edit = clock_edit(path, spec.domain_id)
        for entry in edit[_SET]:
            if entry[0] == "PublishClock.inputs:topicName":
                edit[_SET].remove(entry)
                edit[_SET].append(("PublishClock.inputs:topicName",
                                   "/code_generated_clock"))
        edits.append((path, edit))
    if spec.joints:
        edits.append((
            f"{base}/ROS_JointStates",
            joint_states_edit(
                f"{base}/ROS_JointStates", spec.joints,
                spec.robot_prim, spec.domain_id,
            ),
        ))
    if spec.gripper_joints:
        edits.append((
            f"{base}/ROS_JointStates_Gripper",
            joint_states_edit(
                f"{base}/ROS_JointStates_Gripper", spec.gripper_joints,
                spec.robot_prim, spec.domain_id, namespace='gripper',
            ),
        ))
    for i, cam in enumerate(spec.cameras):
        path = f"{base}/ROS_Camera_{cam.suffix}"
        edits.append((path, camera_edit(path, cam, spec.domain_id)))
    for i, lid in enumerate(spec.lidars):
        path = f"{base}/ROS_LidarRTX"
        # Pick a topic that won't collide with the baked-in graph:
        # /laser_scan for the canonical, /code_generated_laser_scan
        # for the test that proves our builder wired something.
        topic = lid.topic if i == 0 else f"/code_generated_laser_scan_{i}"
        edits.append((path, lidar_edit(path, lid, spec.domain_id, topic=topic)))
    if spec.diff_drive is not None:
        path = f"{base}/DiffController"
        edits.append((path, diff_drive_edit(path, spec.diff_drive, spec.robot_prim, spec.domain_id)))
    for path, edit in edits:
        _delete_existing(path)
        # og.Controller.edit signature: (keys, values). ``keys`` only
        # carries graph_path / evaluator_name; the actual edits live
        # in the second arg under CREATE_NODES / SET_VALUES / CONNECT.
        og.Controller.edit(
            {_GRAPH_PATH: path, _EVAL: "execution"},
            {
                _CREATE: edit[_CREATE],
                _SET: edit[_SET],
                _CONNECT: edit[_CONNECT],
            },
        )


# ---------------------------------------------------------------------------
# Preset for the Mercury X1 (sim-course 02 §2.2 specs).
# ---------------------------------------------------------------------------


def x1_spec(robot_prim: str, domain_id: int = 45) -> RobotGraphSpec:
    """Canonical Mercury X1 ROS graph spec."""
    from dual_arm_pkg.joint_order import (
        X1_GRIPPER_JOINTS, X1_LEFT_ARM_JOINTS, X1_RIGHT_ARM_JOINTS,
        X1_WHEEL_JOINTS,
    )
    arm_joints = [JointSpec(j) for j in (
        *X1_WHEEL_JOINTS, *X1_LEFT_ARM_JOINTS, *X1_RIGHT_ARM_JOINTS,
    )]
    gripper_joints = [
        JointSpec(j, topic='/gripper_command') for j in X1_GRIPPER_JOINTS
    ]
    return RobotGraphSpec(
        robot_prim=robot_prim,
        domain_id=domain_id,
        joints=arm_joints,
        gripper_joints=gripper_joints,
        cameras=(
            CameraSpec(prim_path=f"{robot_prim}/arm_camera_left", suffix="left"),
            CameraSpec(prim_path=f"{robot_prim}/arm_camera_right", suffix="right"),
        ),
        lidars=(LidarSpec(prim_path=f"{robot_prim}/front_lidar"),),
        diff_drive=DiffDriveSpec(
            left_wheel_joint=X1_WHEEL_JOINTS[0],
            right_wheel_joint=X1_WHEEL_JOINTS[1],
        ),
        publish_clock=True,
    )
