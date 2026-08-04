#!/usr/bin/env bash
# Safe operator helper for ROBOTAC stage-2 demonstration capture.
# This script never starts a mission unless CONFIRM_FLIGHT=YES is set.
set -o pipefail

REPO_ROOT="${REPO_ROOT:-/var/workspace/docker/isaac/workspace}"
RAW_DIR="${REPO_ROOT}/videos_raw"
OUT_DIR="${REPO_ROOT}/videos"
STATE_DIR="${STAGE2_STATE_DIR:-/tmp/robotac-stage2-demo}"
DISPLAY_ID="${DISPLAY:-:99}"
DRY_RUN="${DRY_RUN:-0}"

mkdir -p "${RAW_DIR}" "${OUT_DIR}" "${STATE_DIR}" "${REPO_ROOT}/bags"

usage() {
  cat <<'USAGE'
Usage: stage2_demo_control.sh <command> [args]

Read-only checks:
  status                 Show scene, PX4, ROS graph, and recording status.
  audit                  Run chain_status and write a timestamped interface report.

Operator actions:
  host                   Launch the full host chain with a timestamped rosbag.
  close-doors            Close both real scene cargo doors and request status.
                         Requires CONFIRM_SCENE_COMMAND=YES.
  release-payload        Open the real bottom door for the isolated task-5 demo.
                         Requires CONFIRM_CARGO_RELEASE=YES.
  mission                Publish ground COMPLETE once; orchestrator dispatches once.
                         Requires CONFIRM_FLIGHT=YES.
  land                   Send the public LAND safety action.
  abort                  Send the public ABORT safety action.

Video capture:
  record-start <1..6|full>
                         Start VNC capture with the required stage-2 filename.
  record-stop            Stop the active capture cleanly.
  record-status          Show the active capture and raw output path.

Set DRY_RUN=1 to print mutating commands without executing them.
The caller must use ROS_DOMAIN_ID=45, rmw_fastrtps_cpp, and UDPv4 Fast DDS.
USAGE
}

print_cmd() {
  printf '+ '
  printf '%q ' "$@"
  printf '\n'
}

run_mutating() {
  print_cmd "$@"
  if [[ "${DRY_RUN}" != "1" ]]; then
    "$@"
  fi
}

source_ros() {
  cd "${REPO_ROOT}"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source install/setup.bash

  if [[ "${ROS_DOMAIN_ID:-}" != "45" ]]; then
    echo "ERROR: ROS_DOMAIN_ID must be 45 (got '${ROS_DOMAIN_ID:-unset}')." >&2
    return 2
  fi
  if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_fastrtps_cpp" ]]; then
    echo "ERROR: RMW_IMPLEMENTATION must be rmw_fastrtps_cpp." >&2
    return 2
  fi
  if [[ "${FASTDDS_BUILTIN_TRANSPORTS:-}" != "UDPv4" ]]; then
    echo "ERROR: FASTDDS_BUILTIN_TRANSPORTS must be UDPv4." >&2
    return 2
  fi
}

record_pid_file="${STATE_DIR}/ffmpeg.pid"
record_path_file="${STATE_DIR}/ffmpeg.path"
record_log_file="${STATE_DIR}/ffmpeg.log"

record_status() {
  if [[ ! -f "${record_pid_file}" ]]; then
    echo "No stage-2 recording is active."
    return 1
  fi
  local pid
  pid="$(<"${record_pid_file}")"
  if kill -0 "${pid}" 2>/dev/null; then
    echo "Recording PID: ${pid}"
    echo "Raw output: $(<"${record_path_file}")"
    echo "ffmpeg log: ${record_log_file}"
    return 0
  fi
  echo "Stale recording state found for PID ${pid}." >&2
  return 1
}

status() {
  source_ros
  echo '=== Isaac/Pegasus ==='
  systemctl show isaacsim51-scene.service \
    -p ActiveState -p SubState -p MainPID --no-pager 2>/dev/null || true
  pgrep -af 'scene_app.py.*--world' || true
  ss -lntp 2>/dev/null | grep ':4560' || true

  echo '=== PX4 ==='
  docker ps --filter name=px4-sitl \
    --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' 2>/dev/null || true
  docker logs --tail 30 px4-sitl 2>&1 \
    | grep -E 'Simulator connected|Ready for takeoff|ERROR|WARN' || true

  echo '=== ROS graph ==='
  ros2 node list 2>/dev/null | sort || true
  ros2 topic list 2>/dev/null \
    | grep -E '^/(clock|drone0|fmu|drone/navigation|cargo_bay|arena)/' \
    | sort || true
  echo '=== Required live endpoints ==='
  for topic in \
    /clock \
    /drone0/state/pose \
    /fmu/out/vehicle_status_v1 \
    /drone/navigation/state \
    /cargo_bay/status; do
    echo "--- ${topic}"
    ros2 topic info "${topic}" 2>/dev/null || true
  done

  echo '=== Recording ==='
  record_status || true
}

host() {
  source_ros
  if ros2 node list 2>/dev/null | grep -qx '/trajectory_executor'; then
    echo 'ERROR: /trajectory_executor is already running; refusing a duplicate host chain.' >&2
    return 3
  fi
  local timestamp bag_output report_output
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  bag_output="${REPO_ROOT}/bags/stage2_demo_${timestamp}"
  report_output="${STATE_DIR}/interface_report_${timestamp}.json"
  echo "Bag output: ${bag_output}"
  echo "Interface report: ${report_output}"
  print_cmd ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py \
    record_bag:=true bag_output:="${bag_output}" \
    interface_report_path:="${report_output}"
  if [[ "${DRY_RUN}" != "1" ]]; then
    exec ros2 launch bridge_competition_pkg host_bridge_bringup.launch.py \
      record_bag:=true bag_output:="${bag_output}" \
      interface_report_path:="${report_output}"
  fi
}

audit() {
  source_ros
  local timestamp report_output
  timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
  report_output="${STATE_DIR}/interface_report_${timestamp}.json"
  ros2 run bridge_competition_pkg chain_status
  ros2 run bridge_competition_pkg drone_interface_audit --ros-args \
    -p report_path:="${report_output}" -p backend_mode:=px4
  echo "Interface report: ${report_output}"
}

close_doors() {
  source_ros
  if [[ "${CONFIRM_SCENE_COMMAND:-}" != "YES" ]]; then
    echo 'ERROR: set CONFIRM_SCENE_COMMAND=YES to operate the real scene doors.' >&2
    return 4
  fi
  run_mutating ros2 topic pub --once /cargo_bay/command std_msgs/msg/String \
    "{data: 'left_close'}"
  run_mutating ros2 topic pub --once /cargo_bay/command std_msgs/msg/String \
    "{data: 'bottom_close'}"
  run_mutating ros2 topic pub --once /cargo_bay/command std_msgs/msg/String \
    "{data: 'status'}"
}

release_payload() {
  source_ros
  if [[ "${CONFIRM_CARGO_RELEASE:-}" != "YES" ]]; then
    echo 'ERROR: set CONFIRM_CARGO_RELEASE=YES to open the real bottom door.' >&2
    return 4
  fi
  run_mutating ros2 topic pub --once /cargo_bay/command std_msgs/msg/String \
    "{data: 'bottom_open'}"
}

mission() {
  source_ros
  if [[ "${CONFIRM_FLIGHT:-}" != "YES" ]]; then
    echo 'ERROR: set CONFIRM_FLIGHT=YES to dispatch the autonomous flight.' >&2
    return 5
  fi
  echo 'Dispatching one COMPLETE ground-state event.'
  echo 'The orchestrator auto-dispatches; mission_trigger is intentionally not run.'
  run_mutating ros2 run bridge_competition_pkg ground_state_sim --state COMPLETE
}

safety_action() {
  local command="$1"
  local command_id
  case "${command}" in
    LAND) command_id=4 ;;
    ABORT) command_id=5 ;;
    *) echo "ERROR: unsupported safety action ${command}." >&2; return 2 ;;
  esac
  source_ros
  run_mutating ros2 action send_goal /drone/flight_command \
    grasp_demo_interfaces/action/DroneFlightCommand \
    "{command: ${command_id}, position_tolerance: 0.2}"
}

record_start() {
  local task="${1:-}"
  local label
  case "${task}" in
    1) label='仿真基础' ;;
    2) label='关侧舱' ;;
    3) label='起飞飞行' ;;
    4) label='视觉对准' ;;
    5) label='投放执行' ;;
    6) label='精准度' ;;
    full) label='全流程' ;;
    *) echo 'ERROR: record-start requires a task number from 1 to 6, or full.' >&2; return 2 ;;
  esac
  if record_status >/dev/null 2>&1; then
    echo 'ERROR: a recording is already active; run record-stop first.' >&2
    return 6
  fi
  if ! command -v ffmpeg >/dev/null 2>&1; then
    echo 'ERROR: ffmpeg is not installed.' >&2
    return 7
  fi

  local timestamp output
  local -a encoder_args
  timestamp="$(date -u +%Y%m%d_%H%M%S)"
  if [[ "${task}" == "full" ]]; then
    output="${RAW_DIR}/${timestamp}-预选赛加分1-${label}_raw.mp4"
  else
    output="${RAW_DIR}/${timestamp}-预选赛赛段2任务${task}-${label}_raw.mp4"
  fi
  if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q 'h264_nvenc'; then
    # NVENC uses the T4's dedicated encoder and avoids starving Isaac/RTX
    # LiDAR with a multi-core libx264 capture during precision landing.
    encoder_args=(-c:v h264_nvenc -preset p4 -cq 23 -b:v 0)
  else
    encoder_args=(-c:v libx264 -preset veryfast -crf 23)
  fi
  print_cmd ffmpeg -nostdin -f x11grab -video_size 1920x1080 -framerate 30 \
    -i "${DISPLAY_ID}" "${encoder_args[@]}" \
    -pix_fmt yuv420p "${output}"
  if [[ "${DRY_RUN}" == "1" ]]; then
    return 0
  fi

  nohup ffmpeg -nostdin -hide_banner -loglevel warning -y \
    -f x11grab -video_size 1920x1080 -framerate 30 -i "${DISPLAY_ID}" \
    "${encoder_args[@]}" -pix_fmt yuv420p \
    "${output}" >"${record_log_file}" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${record_pid_file}"
  printf '%s\n' "${output}" >"${record_path_file}"
  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "ERROR: ffmpeg exited; inspect ${record_log_file}." >&2
    rm -f "${record_pid_file}" "${record_path_file}"
    return 8
  fi
  record_status
}

record_stop() {
  if [[ ! -f "${record_pid_file}" ]]; then
    echo 'ERROR: no recording is active.' >&2
    return 1
  fi
  local pid output
  pid="$(<"${record_pid_file}")"
  output="$(<"${record_path_file}")"
  if kill -0 "${pid}" 2>/dev/null; then
    print_cmd kill -INT "${pid}"
    if [[ "${DRY_RUN}" != "1" ]]; then
      kill -INT "${pid}"
      for _ in $(seq 1 20); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.25
      done
    fi
  fi
  if [[ "${DRY_RUN}" != "1" ]]; then
    rm -f "${record_pid_file}" "${record_path_file}"
  fi
  echo "Raw recording: ${output}"
  echo "Post-process with: bash docs/project/postprocess_videos.sh <TEAM_NAME>"
}

command="${1:-}"
shift || true
case "${command}" in
  status) status ;;
  audit) audit ;;
  host) host ;;
  close-doors) close_doors ;;
  release-payload) release_payload ;;
  mission) mission ;;
  land) safety_action 4 ;;
  abort) safety_action 5 ;;
  record-start) record_start "$@" ;;
  record-stop) record_stop ;;
  record-status) record_status ;;
  help|-h|--help|'') usage ;;
  *) echo "Unknown command: ${command}" >&2; usage >&2; exit 2 ;;
esac
