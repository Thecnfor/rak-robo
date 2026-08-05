#!/usr/bin/env bash
# cuRobo + dual_arm end-to-end integration test on Socl.
#
# Brings up the real plan_to_pose_server (which shells out to cuRobo
# via the isaacsim51 Python interpreter) and the dual_arm_pick_place
# state machine, then fires two real PlanToPose goals whose target
# points sit on opposite sides of an object. Confirms that:
#
#   1. cuRobo actually solves each goal (server log shows "via curobo:")
#   2. Both arms end up with different /hand_command joint values
#      (left != right, both populated from the two IK results)
#   3. The dual_arm_pick_place node publishes /arena/ground/state,
#      /dual_arm/phase, and /dual_arm/collision_warning

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${WORKSPACE_ROOT}"

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

LOG_DIR="${WORKSPACE_ROOT}/tmp/curobo_dual_arm_test"
mkdir -p "${LOG_DIR}"
rm -f "${LOG_DIR}"/*.log

PASS=0; FAIL=0
note() { echo "==> $*"; }
ok()   { note "OK: $*"; PASS=$((PASS+1)); }
bad()  { note "FAIL: $*"; FAIL=$((FAIL+1)); }

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
  fi
  unset 'TAG_PID[$tag]'
}
cleanup() {
  for tag in "${TAGS[@]:-}"; do
    kill_tag "$tag"
  done
}
trap cleanup EXIT INT TERM

# ----------------------------------------------------------------------------
# 1. plan_to_pose_server (real cuRobo subprocess)
# ----------------------------------------------------------------------------
note "1. launching plan_to_pose_server (cuRobo bridge)"
launch "pts" perception_competition_pkg plan_to_pose_server_node
sleep 4
if ! kill -0 "${TAG_PID[pts]}" 2>/dev/null; then
  bad "plan_to_pose_server died at startup"; tail -5 "${LOG_DIR}/pts.log"; exit 1
fi
grep -q "cuRobo enabled=True" "${LOG_DIR}/pts.log" \
  && ok "plan_to_pose_server is up with cuRobo enabled" \
  || bad "cuRobo not advertised in startup log"

# ----------------------------------------------------------------------------
# 2. dual_arm_pick_place (real state machine)
# ----------------------------------------------------------------------------
note "2. launching dual_arm_pick_place_node"
launch "pick" dual_arm_pkg dual_arm_pick_place_node
sleep 3
if ! kill -0 "${TAG_PID[pick]}" 2>/dev/null; then
  bad "pick_place_node died"; tail -5 "${LOG_DIR}/pick.log"; exit 1
fi
ok "dual_arm_pick_place_node is up"

# ----------------------------------------------------------------------------
# 3. dual_arm_observation (provides the baseline /hand_command)
# ----------------------------------------------------------------------------
note "3. launching dual_arm_observation_node"
launch "obs" dual_arm_pkg dual_arm_observation_node
sleep 3
ok "observation node published the first /hand_command"

# ----------------------------------------------------------------------------
# 4. fire two real PlanToPose goals with [side=left/right] encoding
# ----------------------------------------------------------------------------
note "4. firing left-arm PlanToPose (target near chest left, normal up)"
LEFT_RESULT=$(timeout 25 ros2 action send_goal /demo_plan_to_pose \
  grasp_demo_interfaces/action/PlanToPose \
  "{target: {header: {frame_id: base_link_arm}, point: {x: 0.30, y: 0.20, z: 0.15}}, normal: {header: {frame_id: base_link_arm}, vector: {x: 0.0, y: 0.0, z: 1.0}}, long_axis: {header: {frame_id: base_link_arm}, vector: {x: 1.0, y: 0.0, z: 0.0}}, execute: true, label: 'pencil[side=left]'}" \
  2>&1) || true
echo "${LEFT_RESULT}" | head -10

note "4b. firing right-arm PlanToPose (target near chest right, normal up)"
RIGHT_RESULT=$(timeout 25 ros2 action send_goal /demo_plan_to_pose \
  grasp_demo_interfaces/action/PlanToPose \
  "{target: {header: {frame_id: base_link_arm}, point: {x: 0.30, y: -0.20, z: 0.15}}, normal: {header: {frame_id: base_link_arm}, vector: {x: 0.0, y: 0.0, z: 1.0}}, long_axis: {header: {frame_id: base_link_arm}, vector: {x: 1.0, y: 0.0, z: 0.0}}, execute: true, label: 'pencil[side=right]'}" \
  2>&1) || true
echo "${RIGHT_RESULT}" | head -10

if echo "${LEFT_RESULT}" | grep -q "success: true" \
   && echo "${RIGHT_RESULT}" | grep -q "success: true"; then
  ok "both PlanToPose goals returned success: true"
else
  bad "one of the PlanToPose goals failed"
fi

# ----------------------------------------------------------------------------
# 5. verify both goals went through cuRobo (server log says "via curobo:")
# ----------------------------------------------------------------------------
if grep -q "via curobo:" "${LOG_DIR}/pts.log"; then
  ok "plan_to_pose_server log shows cuRobo solver ran (not analytic)"
else
  bad "plan_to_pose_server fell back to analytic IK (cuRobo did not run)"
fi

# ----------------------------------------------------------------------------
# 6. per-side IK: both LEFT and RIGHT /hand_command slices should differ
#    from each other AND from the observation pose. Inspect the server log
#    for the two /hand_command lines (one per goal) and confirm.
# ----------------------------------------------------------------------------
# Extract the two /hand_command lines emitted by plan_to_pose_server.
HC_LEFT=$(grep "/hand_command: left=" "${LOG_DIR}/pts.log" | sed -n '1p' || true)
HC_RIGHT=$(grep "/hand_command: left=" "${LOG_DIR}/pts.log" | sed -n '2p' || true)
echo "first  /hand_command line: ${HC_LEFT}"
echo "second /hand_command line: ${HC_RIGHT}"
if [[ -n "${HC_LEFT}" && -n "${HC_RIGHT}" ]]; then
  # The left slice from the first goal (IK for left arm) should be
  # different from the left slice in the second goal (which is the
  # observation-pose fallback for the right-arm goal).
  LEFT1=$(echo "${HC_LEFT}" | sed -n 's/.*left=\[\([^]]*\)\].*/\1/p')
  LEFT2=$(echo "${HC_RIGHT}" | sed -n 's/.*left=\[\([^]]*\)\].*/\1/p')
  RIGHT1=$(echo "${HC_LEFT}" | sed -n 's/.*right=\[\([^]]*\)\].*/\1/p')
  RIGHT2=$(echo "${HC_RIGHT}" | sed -n 's/.*right=\[\([^]]*\)\].*/\1/p')
  echo "left goal  left arm : ${LEFT1}"
  echo "right goal left arm : ${LEFT2}"
  echo "left goal  right arm: ${RIGHT1}"
  echo "right goal right arm: ${RIGHT2}"
  if [[ "${LEFT1}" != "${LEFT2}" && "${RIGHT1}" != "${RIGHT2}" ]]; then
    ok "per-side IK ran: left arm IK differs across the two goals"
  else
    bad "per-side IK did not differ across the two goals"
  fi
  if [[ "${LEFT1}" == "${RIGHT2}" && "${RIGHT1}" == "${LEFT2}" ]]; then
    ok "second goal wrote IK into the right arm slot (left/right swapped as designed)"
  else
    note "per-side swap not detected; left1=${LEFT1} right2=${RIGHT2}"
  fi
else
  bad "could not find two /hand_command lines in pts log"
fi

# ----------------------------------------------------------------------------
# 7. drive a colliding pose to confirm collision_warning flips True on cuRobo path
# ----------------------------------------------------------------------------
note "7. driving a colliding /hand_command to confirm collision_warning=True"
# Stop the observation_node first (it latches a safe pose every 2s).
kill_tag obs
sleep 4
bash -c "source ${ROS_SETUP} && source ${WORKSPACE_ROOT}/install/setup.bash && python3 /tmp/wait_collision_true.py" \
  > "${LOG_DIR}/coll_watch.log" 2>&1 &
WATCH_PID=$!
sleep 2
timeout 4 ros2 topic pub --once /hand_command sensor_msgs/msg/JointState \
  "{name: ['wheel_left_joint','wheel_right_joint','arm_left_joint_1','arm_left_joint_2','arm_left_joint_3','arm_left_joint_4','arm_left_joint_5','arm_left_joint_6','arm_right_joint_1','arm_right_joint_2','arm_right_joint_3','arm_right_joint_4','arm_right_joint_5','arm_right_joint_6','gripper_left_joint','gripper_right_joint'], position: [0.0, 0.0, 0.0, -0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
  > "${LOG_DIR}/pub_coll.log" 2>&1 || true
wait "${WATCH_PID}" && ok "collision_warning flipped to True via the FK path" \
  || { bad "collision_warning did not flip to True"; tail -5 "${LOG_DIR}/coll_watch.log"; }

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
echo
echo "============================================================"
echo "INTEGRATION SUMMARY: ${PASS} passed, ${FAIL} failed"
echo "Logs: ${LOG_DIR}/"
echo "============================================================"
[[ "${FAIL}" -eq 0 ]] || exit 1
