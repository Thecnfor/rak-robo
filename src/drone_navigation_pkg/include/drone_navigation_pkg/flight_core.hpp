// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace drone_navigation
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs);
Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs);
Vec3 operator*(const Vec3 & value, double scalar);
Vec3 operator/(const Vec3 & value, double scalar);
double norm(const Vec3 & value);
double distance(const Vec3 & lhs, const Vec3 & rhs);

struct PrearmPoseSample
{
  Vec3 position;
  Vec3 velocity;
  double roll_radians{0.0};
  double pitch_radians{0.0};
};

struct PrearmPoseLimits
{
  Vec3 expected_position;
  double position_tolerance{0.004};
  double max_speed{0.05};
  double max_tilt_radians{0.05235987755982989};
};

bool prearmPoseAllowed(
  const PrearmPoseSample & sample,
  const PrearmPoseLimits & limits);

bool freshPlannerMapReady(
  bool state_received,
  bool map_ready,
  double state_age_seconds,
  double timeout_seconds);

bool fixedSetpointReady(
  bool diagnostic_enabled,
  bool setpoint_received,
  double setpoint_age_seconds,
  double timeout_seconds);

std::uint64_t nextMonotonicTimestampMicros(
  std::uint64_t proposed_timestamp,
  std::uint64_t previous_timestamp);

std::optional<bool> boolTokenValue(
  const std::string & text,
  const std::string & key);

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Px4OdometrySample
{
  std::array<double, 3> position_ned{};
  std::array<double, 3> velocity_ned{};
  std::array<double, 4> attitude_frd_to_ned_wxyz{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity_frd{};
};

struct RosOdometrySample
{
  Vec3 position_enu;
  Vec3 velocity_enu;
  Quaternion attitude_flu_to_enu;
  Vec3 angular_velocity_flu;
};

RosOdometrySample px4NedFrdToRosEnuFlu(const Px4OdometrySample & sample);
Vec3 enuToNed(const Vec3 & value);
double yawEnuToNed(double yaw_enu);

Vec3 visualAlignmentVelocityEnu(
  double normalized_image_x,
  double normalized_image_y,
  double vehicle_yaw_enu,
  double proportional_gain,
  double maximum_speed);

bool visualTargetRecent(
  bool have_valid_detection,
  double valid_detection_age_seconds,
  double loss_grace_seconds);

Vec3 positionAlignmentVelocityEnu(
  const Vec3 & current_position,
  const Vec3 & target_position,
  double proportional_gain,
  double maximum_horizontal_speed,
  double maximum_vertical_speed);

Vec3 rawErrorCorrectedNavigationTarget(
  const Vec3 & raw_target,
  const Vec3 & raw_position,
  const Vec3 & navigation_position,
  double horizontal_gain);

Vec3 stagedReturnRawTarget(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  double transit_clearance,
  double approach_clearance,
  double descent_radius);

bool returnTransitWaypointReached(
  const Vec3 & raw_position,
  const Vec3 & transit_waypoint,
  double horizontal_tolerance,
  double vertical_tolerance);

bool returnFineAlignmentReady(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  double horizontal_radius);

Vec3 returnRouteRawTarget(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  const Vec3 & transit_waypoint,
  const Vec3 & approach_waypoint,
  bool transit_waypoint_reached,
  double fine_alignment_radius,
  double transit_clearance,
  double approach_clearance,
  double descent_radius);

enum class ExecutorSafetyAction {CONTINUE, HOLD, LAND};

ExecutorSafetyAction executorSafetyAction(
  bool trajectory_expected,
  double odometry_age_seconds,
  double control_intent_age_seconds,
  double trajectory_age_seconds,
  double hold_timeout_seconds,
  double land_timeout_seconds);

double trajectoryControlSourceAge(
  bool have_trajectory,
  bool trajectory_started,
  double trajectory_receipt_age_seconds);

double trajectoryReplacementDelay(
  double configured_minimum_seconds,
  double active_trajectory_duration_seconds);

bool trajectoryMinimumAltitudeImproves(
  double current_minimum_altitude,
  double incoming_minimum_altitude,
  double required_improvement);

bool plannerRecoveryAllowsTrajectoryReplacement(
  const std::string & previous_state,
  const std::string & current_state);

bool forcedTrajectoryEndpointChanged(
  bool have_current_trajectory,
  const Vec3 & current_endpoint,
  const Vec3 & incoming_endpoint,
  double endpoint_tolerance);

bool operatorArmRequestAllowed(
  bool have_goal,
  bool side_door_closed,
  bool px4_inputs_ready,
  bool prearm_pose_allowed,
  bool landed_known,
  bool landed,
  bool armed,
  bool offboard);

bool prearmAttitudeAgreementAllowed(
  double raw_roll_radians,
  double raw_pitch_radians,
  double estimated_roll_radians,
  double estimated_pitch_radians,
  double maximum_error_radians);

bool px4DiscreteStateUsable(
  bool state_received,
  double continuous_transport_age_seconds,
  double transport_timeout_seconds);

bool updateSideDoorClosed(bool current_state, const std::string & cargo_status);

bool shouldAcceptTrajectoryUpdate(
  bool trajectory_started,
  bool armed,
  bool offboard,
  double accepted_trajectory_age_seconds,
  double minimum_execution_seconds);

enum class ExecutorRequestedMode
{
  DISABLED,
  ARM_TRAJECTORY,
  TRAJECTORY,
  HOLD,
  VISUAL,
  LAND,
  RESET,
};

enum class ExecutorFlightState
{
  DISABLED,
  PRESTREAM,
  ACTIVE,
  HOLD,
  LAND_LATCHED,
  COMPLETE,
};

bool shouldRequestGroundDisarm(
  ExecutorFlightState state,
  bool armed,
  bool auto_land,
  bool landed,
  bool landed_after_latch,
  double landed_duration_seconds,
  double landing_state_duration_seconds,
  double minimum_ground_delay_seconds);

bool forceDisarmDiagnosticAllowed(
  bool diagnostic_enabled,
  ExecutorFlightState state,
  bool armed,
  bool auto_land);

bool forceDisarmBypassesLandLatch(
  bool diagnostic_enabled,
  bool land_latched,
  const std::string & operator_mode);

bool armCommandAllowed(
  bool arm_requested,
  bool preflight_accepted,
  bool failsafe,
  double previous_command_age_seconds);

ExecutorRequestedMode reduceExecutorRequest(
  ExecutorRequestedMode current,
  ExecutorRequestedMode incoming,
  ExecutorFlightState state);

struct ExecutorLifecycleInputs
{
  ExecutorRequestedMode requested_mode{ExecutorRequestedMode::DISABLED};
  bool prestream_complete{false};
  bool armed{false};
  bool offboard{false};
  bool auto_land{false};
  bool auto_loiter{false};
  bool landed_known{false};
  bool landed{true};
  bool failsafe{false};
};

struct ExecutorLifecycleDecision
{
  ExecutorFlightState state{ExecutorFlightState::DISABLED};
  bool stream_offboard{false};
  bool request_offboard{false};
  bool request_arm{false};
  bool request_land{false};
  bool request_loiter{false};
};

class ExecutorLifecycle
{
public:
  ExecutorLifecycleDecision update(const ExecutorLifecycleInputs & inputs);
  ExecutorFlightState state() const;

private:
  ExecutorFlightState state_{ExecutorFlightState::DISABLED};
};

struct PlannerConfig
{
  double resolution{0.10};
  double inflation_radius{0.25};
  double horizontal_range{5.5};
  double vertical_range{4.5};
  double virtual_ceiling{2.9};
  double max_velocity{0.5};
  double max_acceleration{1.0};
  std::size_t max_expanded_voxels{50000};
  double heuristic_weight{1.5};
};

class RollingVoxelMap
{
public:
  RollingVoxelMap(double resolution, double retention_seconds);
  void update(const std::vector<Vec3> & points, double stamp_seconds);
  std::vector<Vec3> obstaclesAround(
    const Vec3 & center,
    double horizontal_range,
    double vertical_range,
    double stamp_seconds);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class VoxelPlanner
{
public:
  explicit VoxelPlanner(PlannerConfig config = {});
  void setObstacles(const std::vector<Vec3> & points);
  std::vector<Vec3> plan(const Vec3 & start, const Vec3 & goal) const;
  bool collisionFree(const Vec3 & start, const Vec3 & goal) const;

private:
  struct Impl;
  PlannerConfig config_;
  std::shared_ptr<Impl> impl_;
};

struct TrajectoryState
{
  Vec3 position;
  Vec3 velocity;
  Vec3 acceleration;
  double yaw{0.0};
};

struct TimedTrajectoryState
{
  double time_from_start_seconds{0.0};
  TrajectoryState state;
};

std::vector<TimedTrajectoryState> sampleCollisionFreePolyline(
  const std::vector<Vec3> & waypoints,
  double sample_period_seconds,
  double maximum_speed,
  double maximum_acceleration);

struct PositionControlSetpoint
{
  std::array<double, 3> position;
  std::array<double, 3> velocity;
  std::array<double, 3> acceleration;
  double yaw{0.0};
  double yawspeed{0.0};
};

PositionControlSetpoint fixedDiagnosticControlSetpoint(
  const TrajectoryState & state_ned,
  bool vertical_only);

bool verticalOnlyDiagnosticActive(
  bool enabled,
  bool currently_active,
  bool inside_guided_region,
  double current_clearance_m,
  double release_clearance_m,
  double reengage_clearance_m);

double trustedLiftClearance(
  double estimated_clearance_m,
  bool truth_valid,
  double truth_clearance_m);

bool verticalOnlyHandoffConfigurationSafe(
  double release_clearance_m,
  double reengage_clearance_m,
  double physical_guide_height_m,
  double minimum_handoff_lead_m);

double fixedHandoffBlendScale(double elapsed_seconds, double blend_seconds);

class UniformBsplineTrajectory
{
public:
  static UniformBsplineTrajectory fromWaypoints(
    const std::vector<Vec3> & waypoints, double max_velocity, double max_acceleration);
  TrajectoryState sample(double seconds) const;
  double duration() const;
  bool empty() const;

private:
  std::vector<Vec3> waypoints_;
  std::vector<double> segment_start_times_;
  double duration_{0.0};
};

enum class FlightPhase
{
  IDLE,
  PREFLIGHT,
  ARMING,
  TAKEOFF,
  EGO_TRANSIT,
  TARGET_SEARCH,
  VISUAL_ALIGN,
  DROP_HOLD,
  RETURN,
  LAND,
  COMPLETE,
  HOLD,
};

bool shouldResetReturnTransitWaypoint(
  FlightPhase previous_phase,
  FlightPhase current_phase);

std::string toString(FlightPhase phase);

struct SupervisorInputs
{
  bool mission_requested{false};
  bool ground_task_complete{false};
  bool side_door_closed{false};
  bool px4_ready{false};
  bool armed{false};
  bool offboard{false};
  bool px4_failsafe{false};
  bool at_takeoff_pose{false};
  bool at_search_pose{false};
  bool target_visible{false};
  bool target_aligned{false};
  bool payload_released{false};
  bool drop_release_settled{false};
  bool at_home{false};
  bool landed{false};
  double odometry_age_seconds{0.0};
  double pointcloud_age_seconds{0.0};
  double hold_timeout_seconds{0.3};
  double land_timeout_seconds{1.0};
};

struct SupervisorDecision
{
  FlightPhase phase{FlightPhase::IDLE};
  bool command_close_side_door{false};
  bool command_open_bottom_door{false};
  bool request_arm_offboard{false};
  bool request_land{false};
  bool hold_position{false};
  std::string reason;
};

class FlightSupervisor
{
public:
  SupervisorDecision update(const SupervisorInputs & inputs);
  FlightPhase phase() const;

private:
  FlightPhase phase_{FlightPhase::IDLE};
  FlightPhase resume_phase_{FlightPhase::IDLE};
};

}  // namespace drone_navigation
