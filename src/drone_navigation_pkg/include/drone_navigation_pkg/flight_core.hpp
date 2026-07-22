// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
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
  double position_tolerance{0.02};
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

enum class ExecutorSafetyAction {CONTINUE, HOLD, LAND};

ExecutorSafetyAction executorSafetyAction(
  bool trajectory_expected,
  double odometry_age_seconds,
  double control_intent_age_seconds,
  double trajectory_age_seconds,
  double hold_timeout_seconds,
  double land_timeout_seconds);

bool operatorArmRequestAllowed(
  bool have_goal,
  bool side_door_closed,
  bool px4_inputs_ready,
  bool prearm_pose_allowed,
  bool landed_known,
  bool landed,
  bool armed,
  bool offboard);

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
};

}  // namespace drone_navigation
