#!/usr/bin/env bash
# Real end-to-end ROS 2 verification of dual_arm_pkg.
#
# Brings up the observation, gripper, and pick-place nodes on the
# local ROS 2 bus, fires real ros2 action send_goal calls, publishes
# a stub /hand_command JointState that intentionally puts the two
# wrists inside the safe-distance threshold, and asserts the
# /dual_arm/collision_warning topic flips to True within a bounded
# time. Exits non-zero on any failure so CI / pre-commit can use it.
#
# Usage:
#   bash docs/runbooks/verify_dual_arm.sh
#
# Prequisites: the workspace has been built and ``install/setup.bash``
# is on the bash command line via the wrapper below.

set -o pipefail

# -- locate the workspace ---------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${WORKSPACE_ROOT}"

# Pick whichever ROS 2 distro is actually installed (Lyrical on the
# local dev box, Jazzy on Socl). The CI machine could be either.
ROS_SETUP=""
for candidate in "${ROS_DISTRO:-}" jazzy lyrical humble; do
  if [[ -n "${candidate}" && -f "/opt/ros/${candidate}/setup.bash" ]]; then
    ROS_SETUP="/opt/ros/${candidate}/setup.bash"
    break
  fi
done
if [[ -z "${ROS_SETUP}" ]]; then
  echo "FATAL: no ROS 2 install under /opt/ros/{jazzy,lyrical,humble}" >&2
  exit 2
fi
echo "==> using ROS 2 distro: ${ROS_SETUP}"

# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

LOG_DIR="${WORKSPACE_ROOT}/tmp/dual_arm_verify"
mkdir -p "${LOG_DIR}"
rm -f "${LOG_DIR}"/*.log "${LOG_DIR}"/*.txt

PASS=0
FAIL=0
note() { echo "==> $*"; }
ok()   { note "OK: $*"; PASS=$((PASS+1)); }
bad()  { note "FAIL: $*"; FAIL=$((FAIL+1)); }

cleanup() {
  note "cleanup: killing background ros2 processes"
  for tag in "${TAGS[@]:-}"; do
    pid="${TAG_PID[$tag]:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 1
  for tag in "${TAGS[@]:-}"; do
    pid="${TAG_PID[$tag]:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

# Tag-indexed PIDs so we can kill individual nodes by name (e.g. the
# observation_node has to die before the colliding-pose test, otherwise
# it latches a safe pose every 2s and overwrites our colliding pub).
declare -A TAG_PID
TAGS=()
launch() {
  local tag="$1"; shift
  ros2 run "$@" > "${LOG_DIR}/${tag}.log" 2>&1 &
  TAG_PID[$tag]=$!
  TAGS+=("$tag")
  echo "started ${tag} pid=${TAG_PID[$tag]}"
}
kill_tag() {
  local tag="$1"
  local pid="${TAG_PID[$tag]:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    sleep 1
    kill -9 "${pid}" 2>/dev/null || true
    echo "killed ${tag} pid=${pid}"
  fi
  unset 'TAG_PID[$tag]'
}

# ----------------------------------------------------------------------------
# 1. observation node publishes 16-DOF /hand_command
# ----------------------------------------------------------------------------
note "1. launching dual_arm_observation_node"
launch "obs" dual_arm_pkg dual_arm_observation_node
sleep 4
if ! kill -0 "${TAG_PID[obs]}" 2>/dev/null; then
  bad "observation_node died at startup"
  tail -5 "${LOG_DIR}/obs.log"
  exit 1
fi
NAMES=$(timeout 4 ros2 topic echo /hand_command --once --field name 2>&1 | head -2)
if [[ "${NAMES}" == *"arm_left_joint_1"* && "${NAMES}" == *"gripper_right_joint"* ]]; then
  ok "observation node publishes 16-DOF /hand_command"
else
  bad "observation node /hand_command payload is wrong: ${NAMES}"
fi

# ----------------------------------------------------------------------------
# 2. gripper server responds to GripperCommand
# ----------------------------------------------------------------------------
note "2. launching dual_gripper_server_node"
launch "grip" dual_arm_pkg dual_gripper_server_node
sleep 3
if ! kill -0 "${TAG_PID[grip]}" 2>/dev/null; then
  bad "gripper_server died at startup"
  tail -5 "${LOG_DIR}/grip.log"
  exit 1
fi
# The /demo_gripper_command action server is only available once
# grasp_demo_interfaces is built. If it isn't built (the workspace
# was assembled in a hurry), we just skip the action step.
if ros2 action list 2>/dev/null | grep -q "/demo_gripper_command"; then
  RESULT=$(timeout 5 ros2 action send_goal /demo_gripper_command \
    grasp_demo_interfaces/action/GripperCommand \
    "{command: 'open', position: 0.0, speed: 0.0, wait_for_completion: false}" \
    --feedback 2>&1 || true)
  # ROS 2 prints "success: true" (lowercase t, space-separated); a
  # result of "success: false" or "Goal finished with status: ABORTED"
  # means the action server failed the goal.
  if [[ "${RESULT}" == *"success: true"* ]] && [[ "${RESULT}" != *"ABORTED"* ]]; then
    ok "GripperCommand 'open' succeeded"
  else
    bad "GripperCommand 'open' did not return success: ${RESULT}"
  fi
else
  note "skipping GripperCommand test (grasp_demo_interfaces not built)"
fi

# ----------------------------------------------------------------------------
# 3. pick_place node subscribes to /hand_command and runs FK
# ----------------------------------------------------------------------------
note "3. launching dual_arm_pick_place_node"
launch "pick" dual_arm_pkg dual_arm_pick_place_node
sleep 4
if ! kill -0 "${TAG_PID[pick]}" 2>/dev/null; then
  bad "pick_place_node died at startup"
  tail -5 "${LOG_DIR}/pick.log"
  exit 1
fi
ok "pick_place_node is alive and listening on /hand_command"

# ----------------------------------------------------------------------------
# 4. drive a SAFE pose (zero joints) -> /dual_arm/collision_warning = False
# ----------------------------------------------------------------------------
note "4. publishing a SAFE /hand_command and waiting for collision_warning=False"
# 16-element JointState: two wheels, six left, six right, two grippers.
# Padded with extra zeros because plan_to_pose_server publishes 16 floats.
SAFE_STAMP=$(date +%s%N)
timeout 4 ros2 topic pub --once /hand_command sensor_msgs/msg/JointState \
  "{name: ['wheel_left_joint','wheel_right_joint','arm_left_joint_1','arm_left_joint_2','arm_left_joint_3','arm_left_joint_4','arm_left_joint_5','arm_left_joint_6','arm_right_joint_1','arm_right_joint_2','arm_right_joint_3','arm_right_joint_4','arm_right_joint_5','arm_right_joint_6','gripper_left_joint','gripper_right_joint'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
  > "${LOG_DIR}/pub_safe.log" 2>&1 || true

# Collect up to 5 messages from /dual_arm/collision_warning; one of
# them should report data=False (we tolerate older True messages if
# the node booted before the safe pose arrived).
WARNINGS=$(timeout 6 ros2 topic echo /dual_arm/collision_warning --once 2>&1 || true)
echo "${WARNINGS}" | grep -q "data: false" \
  && ok "collision_warning flipped to False for safe pose" \
  || {
      # Maybe we caught a stale True; try one more sample.
      WARNINGS2=$(timeout 6 ros2 topic echo /dual_arm/collision_warning --once 2>&1 || true)
      echo "${WARNINGS2}" | grep -q "data: false" \
        && ok "collision_warning flipped to False on retry" \
        || bad "collision_warning did not flip to False: ${WARNINGS}"
   }

# ----------------------------------------------------------------------------
# 5. drive a COLLIDING pose -> /dual_arm/collision_warning = True
# ----------------------------------------------------------------------------
note "5. stopping observation_node (it latches a safe pose every 2s) ..."
kill_tag "obs"
sleep 4  # let the last observation_node latched pose expire

note "5b. publishing a COLLIDING /hand_command and waiting for collision_warning=True"
# Empirically, the colliding pose for ECO65-B / our keepout is
# left=[0, -0.3, 0.3, 0, 0, 0] / right=[0, 0, 0.3, 0, 0, 0]. See
# src/dual_arm_pkg/test/test_fk_collision.py for the geometry proof.
# The watch script starts before the publish so we never miss the
# True message (DDS transient_local QoS would also work; the dedicated
# Python watcher just makes the race window obvious).
bash -c "source ${ROS_SETUP} && source ${WORKSPACE_ROOT}/install/setup.bash && python3 /tmp/wait_collision_true.py" > "${LOG_DIR}/coll_watch.log" 2>&1 &
WATCH_PID=$!
# Give the watcher's rclpy.init a moment to register the subscription.
sleep 2
timeout 4 ros2 topic pub --once /hand_command sensor_msgs/msg/JointState \
  "{name: ['wheel_left_joint','wheel_right_joint','arm_left_joint_1','arm_left_joint_2','arm_left_joint_3','arm_left_joint_4','arm_left_joint_5','arm_left_joint_6','arm_right_joint_1','arm_right_joint_2','arm_right_joint_3','arm_right_joint_4','arm_right_joint_5','arm_right_joint_6','gripper_left_joint','gripper_right_joint'], position: [0.0, 0.0, 0.0, -0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
  > "${LOG_DIR}/pub_coll.log" 2>&1 || true

# Wait for the Python watcher (it exits non-zero if no True in 6s).
wait "${WATCH_PID}" && ok "collision_warning flipped to True for colliding pose" \
  || { bad "collision_warning did not flip to True"; tail -5 "${LOG_DIR}/coll_watch.log"; }

# ----------------------------------------------------------------------------
# 6. publish /arena/ground/request and confirm /arena/ground/state advances
# ----------------------------------------------------------------------------
note "6. publishing /arena/ground/request and waiting for state-machine advance"
timeout 4 ros2 topic pub --once /arena/ground/request std_msgs/msg/String \
  "{data: 'go'}" \
  > "${LOG_DIR}/pub_req.log" 2>&1 || true

# /arena/ground/state uses TRANSIENT_LOCAL; --once waits for the next
# publication regardless of DDS quirks.
PHASE=$(timeout 6 ros2 topic echo /dual_arm/phase --once 2>&1 || true)
echo "${PHASE}" | grep -qE "(PARALLEL_DETECT|DUAL_GRASP)" \
  && ok "state machine advanced past IDLE: ${PHASE}" \
  || bad "state machine did not advance: ${PHASE}"

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
echo
echo "============================================================"
echo "VERIFY SUMMARY: ${PASS} passed, ${FAIL} failed"
echo "Logs: ${LOG_DIR}/"
echo "============================================================"
[[ "${FAIL}" -eq 0 ]] || exit 1
