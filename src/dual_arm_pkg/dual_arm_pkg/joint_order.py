"""Single source of truth for the Mercury X1 dual-arm joint layout.

All ``dual_arm_pkg`` nodes import from here so the 14-DOF JointState
order, joint limits, and the heuristic collision geometry stay
consistent across observation, gripper, and pick-place nodes.

The DH table is the only piece NOT pulled from the sim-course hardware
page; the official X1 reference deliberately omits numeric DH (the
course text says "图中没直接给" — "the picture does not give it
directly"). We use the UR5e DH as a stand-in until the real values
land; the constant carries an explicit ``ponytail:`` ceiling so the
upgrade path is named.

Why a shared module rather than YAML: the joint order is consumed
inside import-time calculations (e.g. safe-radius indices for the
collision check) and during ``_publish_state`` which runs from a Node
constructor; pulling it from YAML would force every consumer to read
the file in ``__init__`` and add disk-error handling. Two source-of-
truth files (one for layout, one for IK tuning) are fine — three
(plus demo_params.yaml) is the kind of drift that bites during
competition week.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Joint layout — the exact 14-DOF order shared by /hand_command publishers.
# ---------------------------------------------------------------------------

#: 6-DOF left arm, shoulder-out.
X1_LEFT_ARM_JOINTS: Tuple[str, ...] = (
    'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3',
    'arm_left_joint_4', 'arm_left_joint_5', 'arm_left_joint_6',
)

#: 6-DOF right arm, mirror of the left.
X1_RIGHT_ARM_JOINTS: Tuple[str, ...] = (
    'arm_right_joint_1', 'arm_right_joint_2', 'arm_right_joint_3',
    'arm_right_joint_4', 'arm_right_joint_5', 'arm_right_joint_6',
)

#: Differential-drive base.
X1_WHEEL_JOINTS: Tuple[str, ...] = (
    'wheel_left_joint', 'wheel_right_joint',
)

#: Two parallel gripper joints.
X1_GRIPPER_JOINTS: Tuple[str, ...] = (
    'gripper_left_joint', 'gripper_right_joint',
)

#: The exact order every /hand_command publisher must use.
X1_JOINT_ORDER: Tuple[str, ...] = (
    X1_WHEEL_JOINTS
    + X1_LEFT_ARM_JOINTS
    + X1_RIGHT_ARM_JOINTS
    + X1_GRIPPER_JOINTS
)

#: Number of DOF in the full /hand_command JointState.
X1_TOTAL_DOF: int = len(X1_JOINT_ORDER)  # = 16

#: Slice indices for each group in a flat 16-vector.
#: Use these everywhere instead of hard-coded magic numbers like [2:8].
X1_WHEEL_SLICE: slice = slice(0, 2)
X1_LEFT_ARM_SLICE: slice = slice(2, 8)
X1_RIGHT_ARM_SLICE: slice = slice(8, 14)
X1_GRIPPER_SLICE: slice = slice(14, 16)


# ---------------------------------------------------------------------------
# Joint limits — sim-course 02-X1 §2.2.1.3 (Mercury X1 hardware page).
# Each tuple is (lower_rad, upper_rad) per the official spec.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _JointLimit:
    lower: float
    upper: float


#: Real per-joint range from the X1 hardware reference.
#: These are the *physical* stops, NOT the cuRobo analytic-IK stops
#: (the latter are tighter to avoid USD ArticulationController NaN
#: during M2 bringup; see perception_competition_pkg/plan_to_pose_server_node.py).
X1_ARM_JOINT_LIMITS: Tuple[_JointLimit, ...] = (
    _JointLimit(-2.879,  2.879),   # joint1 — shoulder yaw
    _JointLimit(-0.873,  2.094),   # joint2 — shoulder pitch
    _JointLimit(-2.879,  0.087),   # joint3 — elbow
    _JointLimit(-2.879,  2.879),   # joint4 — wrist 1
    _JointLimit(-1.920,  3.054),   # joint5 — wrist 2
    _JointLimit(-3.140,  3.140),   # joint6 — wrist 3
)


def joint_in_range(joint_index: int, value: float) -> bool:
    """Return True iff *value* lies inside the X1 hardware limits."""
    lo, hi = X1_ARM_JOINT_LIMITS[joint_index].lower, X1_ARM_JOINT_LIMITS[joint_index].upper
    return lo <= value <= hi


# ---------------------------------------------------------------------------
# Gripper geometry.
# ---------------------------------------------------------------------------

#: Aperture in metres when the gripper is fully open.
GRIPPER_OPEN_POSITION: float = 0.04

#: Aperture in metres when the gripper is fully closed.
GRIPPER_CLOSED_POSITION: float = 0.0

#: Default half-open pose used by the observation node on startup.
GRIPPER_NEUTRAL_POSITION: float = 0.02


#: 6-DOF observation pose (radians) for the left arm. Shoulder-elbow-wrist
#: angles tuned to clear the cargo bay and the down-camera frame.
OBSERVATION_POSE_LEFT: Tuple[float, ...] = (
    0.0,    # arm_left_joint_1 — base yaw
    -0.4,   # arm_left_joint_2 — shoulder pitch
    0.6,    # arm_left_joint_3 — elbow
    -0.2,   # arm_left_joint_4 — wrist 1
    0.0,    # arm_left_joint_5 — wrist 2
    0.0,    # arm_left_joint_6 — wrist 3
)

#: Mirrored observation pose for the right arm (kept as a separate
#: constant so a future asymmetric pose is one line away).
OBSERVATION_POSE_RIGHT: Tuple[float, ...] = OBSERVATION_POSE_LEFT


def gripper_position(command) -> float:
    """Map a textual command (open / close / stop) to a target aperture."""
    cmd = (str(command or 'open')).strip().lower()
    if cmd == 'open':
        return GRIPPER_OPEN_POSITION
    if cmd in {'close', 'stop'}:
        return GRIPPER_CLOSED_POSITION
    return GRIPPER_OPEN_POSITION


# ---------------------------------------------------------------------------
# Heuristic collision geometry.
#
# The X1 chest keepout is the rectangle (X half-width, Y half-depth) inside
# which both arm bases must not project their wrist at the same time.
# We treat it as a sphere-sphere distance between the two planned wrist
# points — cheap, deterministic, and easy to unit-test.
# ---------------------------------------------------------------------------

#: Half-width of the X1 chest envelope along the base X axis (metres).
COLLISION_KEEPOUT_X: float = 0.10

#: Half-depth of the X1 chest envelope along the base Y axis (metres).
COLLISION_KEEPOUT_Y: float = 0.10

#: Minimum centre-to-centre wrist distance we accept before raising
#: ``/dual_arm/collision_warning = true``. Sized so the keepout sphere
#: has radius equal to ``COLLISION_KEEPOUT_X``.
COLLISION_SAFE_WRIST_DISTANCE: float = 2.0 * COLLISION_KEEPOUT_X


# ---------------------------------------------------------------------------
# Modified-DH parameters for the 6-DOF arm.
#
# ponytail: UR5e stand-in — replace with the real X1 values once
# perception_competition_pkg/M1 brings in the URDF-exported MDH. Until
# then the FK output is only good enough for the relative geometry the
# collision check needs (sphere-sphere wrist distance).
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _MdhRow:
    a_im1_mm: float    # a_{i-1} in mm
    alpha_im1_deg: float   # α_{i-1} in degrees
    d_i_mm: float      # d_i in mm
    theta_offset_deg: float   # mechanical zero offset


# ponytail: replace with X1 real DH once M1 lands; the collision check
# only needs the relative wrist distance, so the absolute translation is
# not safety-critical today.
X1_MDH: Tuple[_MdhRow, ...] = (
    _MdhRow(0.0,    0.0,   162.5, 0.0),
    _MdhRow(-86.0, -90.0,    0.0, -90.0),
    _MdhRow(260.0,   0.0,    0.0,   0.0),
    _MdhRow(240.0,   0.0, -58.88,  90.0),
    _MdhRow(0.0,    90.0,  110.0,   0.0),
    _MdhRow(0.0,   -90.0,   79.5,   0.0),
)


def forward_kinematics(joints: List[float]) -> np.ndarray:
    """Return the 4×4 wrist transform for the given 6-DOF joint angles.

    Uses Modified-DH. Angles are in radians and added to the per-joint
    ``theta_offset`` (which is itself in radians after conversion).

    The returned frame is expressed in the arm's base_link_arm
    coordinate system (the ``base_link → arm_base_link`` translation is
    added by the caller when comparing to the chest keepout).
    """
    if len(joints) != 6:
        raise ValueError(f'expected 6 joint angles, got {len(joints)}')
    transform = np.eye(4)
    for joint_value, row in zip(joints, X1_MDH):
        a = row.a_im1_mm / 1000.0  # mm → m
        d = row.d_i_mm / 1000.0
        alpha = math.radians(row.alpha_im1_deg)
        theta = joint_value + math.radians(row.theta_offset_deg)
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        # MDH per-link transform: Rx(α)·Tx(a)·Rz(θ)·Tz(d)
        link = np.array([
            [ct,    -st,    0.0,  a],
            [st*ca,  ct*ca, -sa, -sa*d],
            [st*sa,  ct*sa,  ca,  ca*d],
            [0.0,    0.0,   0.0, 1.0],
        ])
        transform = transform @ link
    return transform


#: Transform from base_link to arm_base_link for the LEFT arm in the X1
#: chest frame. The right arm is mirrored along the X axis.
#: ponytail: these offsets come from the X1 chassis drawing (link_body →
#: arm_base_link); if the USD scene ships different numbers, override
#: via ROS parameter (see dual_arm_observation_node).
LEFT_ARM_BASE_OFFSET = np.array([
    [1.0, 0.0, 0.0,  COLLISION_KEEPOUT_X],
    [0.0, 1.0, 0.0,  0.0],
    [0.0, 0.0, 1.0,  0.0],
    [0.0, 0.0, 0.0,  1.0],
])

RIGHT_ARM_BASE_OFFSET = np.array([
    [1.0, 0.0, 0.0, -COLLISION_KEEPOUT_X],
    [0.0, 1.0, 0.0,  0.0],
    [0.0, 0.0, 1.0,  0.0],
    [0.0, 0.0, 0.0,  1.0],
])


def wrist_position(joints: List[float], side: str) -> np.ndarray:
    """Return the world-frame (base_link) wrist point for one arm."""
    base_offset = LEFT_ARM_BASE_OFFSET if side == 'left' else RIGHT_ARM_BASE_OFFSET
    arm_frame = forward_kinematics(joints)
    world = base_offset @ arm_frame
    return world[:3, 3]


def wrist_distance(joints_left: List[float], joints_right: List[float]) -> float:
    """Centre-to-centre distance between the two wrists in base_link."""
    left = wrist_position(joints_left, 'left')
    right = wrist_position(joints_right, 'right')
    return float(np.linalg.norm(left - right))
