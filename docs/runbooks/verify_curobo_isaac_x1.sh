#!/bin/bash
# cuRobo -> Isaac Sim X1 full integration test on Socl.
#
# Runs against the live X1 scene (started by run_demo_scene.sh --world X1
# on display :99). All participants must agree on ROS_DOMAIN_ID=45:
#
#   - X1 USD baked-in ROS2Context nodes (we patched domain 55 -> 45)
#   - plan_to_pose_server (perception_competition_pkg) with cuRobo
#   - this script (or ros2 action send_goal)
#
# Pipeline proven by this script:
#   ros2 action send_goal /demo_plan_to_pose (target pose)
#       |
#       v
#   plan_to_pose_server: invoke_curobo() subprocess
#       |
#       v
#   /hand_command (legacy dual_arm_pkg contract)
#       + /joint_command (X1 USD baked-in subscriber)
#       |
#       v
#   USD ROS_JointStates / ArticulationController
#       |
#       v
#   X1 USD joints actually move
#       |
#       v
#   /joint_states feedback to host
set -e
WS=/var/workspace/docker/isaac/workspace
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=45

# Snapshot the left arm joints (USD puts them around index 4..10 of the
# JointState position vector).
left_arm_state() {
    timeout 2 ros2 topic echo /joint_states --once --field position 2>&1 \
        | head -1 | tr ',' '\n' | sed -n '4,9p' | tr '\n' ' '
}

echo "=== domain: $ROS_DOMAIN_ID ==="
echo "=== joint_states BEFORE ==="
left_arm_state; echo

echo "=== starting plan_to_pose_server ==="
ros2 run perception_competition_pkg plan_to_pose_server_node > /tmp/pts.log 2>&1 &
PTS=$!
sleep 4

echo "=== firing PlanToPose (target 0.40, 0.30, 0.40) ==="
timeout 25 ros2 action send_goal /demo_plan_to_pose \
    grasp_demo_interfaces/action/PlanToPose \
    '{target: {header: {frame_id: base_link_arm}, point: {x: 0.40, y: 0.30, z: 0.40}}, normal: {header: {frame_id: base_link_arm}, vector: {x: 0.0, y: 0.0, z: 1.0}}, long_axis: {header: {frame_id: base_link_arm}, vector: {x: 1.0, y: 0.0, z: 0.0}}, execute: true, label: "pencil[side=left]"}' \
    2>&1 | tail -3

echo "=== cuRobo IK result ==="
grep -E "via curobo|/joint_command|/hand_command" /tmp/pts.log | head -3

echo "=== /joint_states over 6s as X1 chases cuRobo IK ==="
for i in 1 2 3 4 5 6; do
    echo "frame $i:"
    left_arm_state; echo
    sleep 1
done

echo "=== /gripper_joint_states sample ==="
timeout 2 ros2 topic echo /gripper_joint_states --once --field position 2>&1 | head -1

kill -INT "$PTS" 2>/dev/null || true
wait 2>/dev/null || true
echo "=== done ==="
