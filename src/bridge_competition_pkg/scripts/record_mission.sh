#!/usr/bin/env bash
# Run one end-to-end mission under SIH, recording:
#   - rosbag  → /tmp/drone_competition_bag_<ts>/
#   - log     → /tmp/drone_competition_<ts>.log
#   - exit    → chain comes up; the operator stops when land complete
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source /var/workspace/docker/isaac/workspace/install/setup.bash
#   bash src/bridge_competition_pkg/scripts/record_mission.sh
#
# This is the host-side validator used in D-3.3 / D-3.4. The video
# recording for the submission (videos/*.mp4) wraps this with ffmpeg
# screen capture from the VNC display.
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/var/workspace/docker/isaac/workspace}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BAG_DIR="/tmp/drone_competition_bag_${TIMESTAMP}"
LOG_FILE="/tmp/drone_competition_${TIMESTAMP}.log"
REPORT_FILE="/tmp/drone_interface_report_${TIMESTAMP}.json"

export ROS_DOMAIN_ID=45
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${LD_LIBRARY_PATH:-}"

cd "${REPO_ROOT}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "[$(date -Is)] launching host_bridge_bringup"
ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py \
    record_bag:=true interface_report_path:=${REPORT_FILE} \
    > "${LOG_FILE}" 2>&1 &
BRINGUP_PID=$!

# Wait for nodes to come up.
sleep 10

echo "[$(date -Is)] starting cargo_status_sim + ground_state_sim + mission_trigger"
ros2 run bridge_competition_pkg cargo_status_sim >> "${LOG_FILE}" 2>&1 &
CARGO_PID=$!
ros2 run bridge_competition_pkg ground_state_sim >> "${LOG_FILE}" 2>&1 &
GROUND_PID=$!
sleep 1

ros2 run bridge_competition_pkg ground_state_sim >> "${LOG_FILE}" 2>&1 || true
ros2 run bridge_competition_pkg mission_trigger >> "${LOG_FILE}" 2>&1 || true
sleep 30

echo "[$(date -Is)] mission window elapsed; tearing down"

# Tear down: kill the bringup; rosbag stops when bridge_competition_pkg
# exits because ExecuteProcess is a child of the launch.
kill -INT "${BRINGUP_PID}" 2>/dev/null || true
kill -INT "${CARGO_PID}" 2>/dev/null || true
kill -INT "${GROUND_PID}" 2>/dev/null || true
wait "${BRINGUP_PID}" 2>/dev/null || true

echo "[$(date -Is)] bag written to ${BAG_DIR}"
echo "[$(date -Is)] interface report at ${REPORT_FILE}"
echo "[$(date -Is)] full log at ${LOG_FILE}"
