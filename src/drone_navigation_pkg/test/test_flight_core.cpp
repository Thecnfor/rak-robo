// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

using drone_navigation::FlightPhase;
using drone_navigation::FlightSupervisor;
using drone_navigation::ExecutorSafetyAction;
using drone_navigation::ExecutorFlightState;
using drone_navigation::ExecutorLifecycle;
using drone_navigation::ExecutorLifecycleInputs;
using drone_navigation::ExecutorRequestedMode;
using drone_navigation::reduceExecutorRequest;
using drone_navigation::PlannerConfig;
using drone_navigation::Px4OdometrySample;
using drone_navigation::RollingVoxelMap;
using drone_navigation::SupervisorInputs;
using drone_navigation::UniformBsplineTrajectory;
using drone_navigation::Vec3;
using drone_navigation::VoxelPlanner;
using drone_navigation::executorSafetyAction;
using drone_navigation::boolTokenValue;
using drone_navigation::fixedDiagnosticControlSetpoint;
using drone_navigation::fixedHandoffBlendScale;
using drone_navigation::fixedSetpointReady;
using drone_navigation::forcedTrajectoryEndpointChanged;
using drone_navigation::rawErrorCorrectedNavigationTarget;
using drone_navigation::returnRouteRawTarget;
using drone_navigation::returnFineAlignmentReady;
using drone_navigation::returnTransitWaypointReached;
using drone_navigation::shouldResetReturnTransitWaypoint;
using drone_navigation::stagedReturnRawTarget;
using drone_navigation::trajectoryMinimumAltitudeImproves;
using drone_navigation::plannerRecoveryAllowsTrajectoryReplacement;
using drone_navigation::positionAlignmentVelocityEnu;
using drone_navigation::sampleCollisionFreePolyline;
using drone_navigation::trustedLiftClearance;
using drone_navigation::nextMonotonicTimestampMicros;
using drone_navigation::verticalOnlyDiagnosticActive;
using drone_navigation::verticalOnlyHandoffConfigurationSafe;
using drone_navigation::visualAlignmentVelocityEnu;
using drone_navigation::visualTargetRecent;

namespace
{
constexpr double kTolerance = 1e-6;
constexpr double kPi = 3.14159265358979323846;
}

TEST(CoordinateFrames, ConvertsNedFrdToEnuFlu)
{
  Px4OdometrySample px4;
  px4.position_ned = {1.0, 2.0, -3.0};
  px4.velocity_ned = {4.0, 5.0, -6.0};
  px4.angular_velocity_frd = {0.1, 0.2, 0.3};
  px4.attitude_frd_to_ned_wxyz = {1.0, 0.0, 0.0, 0.0};

  const auto ros = drone_navigation::px4NedFrdToRosEnuFlu(px4);
  EXPECT_NEAR(ros.position_enu.x, 2.0, kTolerance);
  EXPECT_NEAR(ros.position_enu.y, 1.0, kTolerance);
  EXPECT_NEAR(ros.position_enu.z, 3.0, kTolerance);
  EXPECT_NEAR(ros.velocity_enu.x, 5.0, kTolerance);
  EXPECT_NEAR(ros.velocity_enu.y, 4.0, kTolerance);
  EXPECT_NEAR(ros.velocity_enu.z, 6.0, kTolerance);
  EXPECT_NEAR(ros.angular_velocity_flu.x, 0.1, kTolerance);
  EXPECT_NEAR(ros.angular_velocity_flu.y, -0.2, kTolerance);
  EXPECT_NEAR(ros.angular_velocity_flu.z, -0.3, kTolerance);

  const double quaternion_norm = std::sqrt(
    ros.attitude_flu_to_enu.x * ros.attitude_flu_to_enu.x +
    ros.attitude_flu_to_enu.y * ros.attitude_flu_to_enu.y +
    ros.attitude_flu_to_enu.z * ros.attitude_flu_to_enu.z +
    ros.attitude_flu_to_enu.w * ros.attitude_flu_to_enu.w);
  EXPECT_NEAR(quaternion_norm, 1.0, kTolerance);
}

TEST(VisualAlignment, RotatesImageErrorIntoEnuWithoutChangingAxes)
{
  const auto yaw_zero = visualAlignmentVelocityEnu(0.4, -0.2, 0.0, 0.5, 0.3);
  EXPECT_NEAR(yaw_zero.x, 0.2, kTolerance);
  EXPECT_NEAR(yaw_zero.y, 0.1, kTolerance);
  EXPECT_NEAR(yaw_zero.z, 0.0, kTolerance);

  const auto yaw_ninety = visualAlignmentVelocityEnu(
    0.4, -0.2, kPi / 2.0, 0.5, 0.3);
  EXPECT_NEAR(yaw_ninety.x, -0.1, kTolerance);
  EXPECT_NEAR(yaw_ninety.y, 0.2, kTolerance);

  const auto saturated = visualAlignmentVelocityEnu(1.0, -1.0, 0.0, 2.0, 0.25);
  EXPECT_NEAR(saturated.x, 0.25, kTolerance);
  EXPECT_NEAR(saturated.y, 0.25, kTolerance);
}

TEST(VisualAlignment, BriefDetectionDropoutUsesOnlyTheLastValidObservation)
{
  EXPECT_TRUE(visualTargetRecent(true, 0.0, 0.6));
  EXPECT_TRUE(visualTargetRecent(true, 0.59, 0.6));
  EXPECT_FALSE(visualTargetRecent(true, 0.61, 0.6));
  EXPECT_FALSE(visualTargetRecent(false, 0.0, 0.6));
  EXPECT_FALSE(visualTargetRecent(
    true, std::numeric_limits<double>::infinity(), 0.6));
  EXPECT_FALSE(visualTargetRecent(true, 0.1, -0.1));
}

TEST(ReturnGuidance, AppliesRawTruthErrorInTheNavigationFrame)
{
  const auto target = rawErrorCorrectedNavigationTarget(
    {4.55, -0.38, 1.185},
    {4.60, -0.35, 1.20},
    {4.58, -0.32, 1.19},
    2.0);
  EXPECT_NEAR(target.x, 4.48, kTolerance);
  EXPECT_NEAR(target.y, -0.38, kTolerance);
  EXPECT_NEAR(target.z, 1.175, kTolerance);
}

TEST(ReturnGuidance, KeepsCruiseClearanceUntilInsideTheDescentRadius)
{
  const Vec3 home{4.55, -0.38, 1.13};
  const auto target = stagedReturnRawTarget(
    home, {5.40, -2.33, 1.80}, 0.67, 0.055, 0.20);

  EXPECT_NEAR(target.x, home.x, kTolerance);
  EXPECT_NEAR(target.y, home.y, kTolerance);
  EXPECT_NEAR(target.z, 1.80, kTolerance);
}

TEST(ReturnGuidance, UsesApproachClearanceOnlyInsideTheDescentRadius)
{
  const Vec3 home{4.55, -0.38, 1.13};
  const auto target = stagedReturnRawTarget(
    home, {4.68, -0.48, 1.80}, 0.67, 0.055, 0.20);

  EXPECT_NEAR(target.x, home.x, kTolerance);
  EXPECT_NEAR(target.y, home.y, kTolerance);
  EXPECT_NEAR(target.z, 1.185, kTolerance);
}

TEST(ReturnGuidance, UsesTransitWaypointBeforeRoutingHome)
{
  const Vec3 home{4.55, -0.38, 1.13};
  const Vec3 waypoint{5.50, -1.70, 1.80};
  const Vec3 approach{4.55, -0.75, 1.80};
  const auto target = returnRouteRawTarget(
    home, {5.51, -3.50, 1.78}, waypoint, approach, false, 0.45,
    0.67, 0.055, 0.20);

  EXPECT_NEAR(target.x, waypoint.x, kTolerance);
  EXPECT_NEAR(target.y, waypoint.y, kTolerance);
  EXPECT_NEAR(target.z, waypoint.z, kTolerance);
}

TEST(ReturnGuidance, RoutesToClearApproachPoseAfterTransitWaypointIsLatched)
{
  const Vec3 home{4.55, -0.38, 1.13};
  const Vec3 waypoint{5.50, -1.70, 1.80};
  const Vec3 approach{4.55, -0.75, 1.80};
  const auto target = returnRouteRawTarget(
    home, {5.49, -1.68, 1.79}, waypoint, approach, true, 0.45,
    0.67, 0.055, 0.20);

  EXPECT_NEAR(target.x, approach.x, kTolerance);
  EXPECT_NEAR(target.y, approach.y, kTolerance);
  EXPECT_NEAR(target.z, approach.z, kTolerance);
}

TEST(ReturnGuidance, RoutesHomeOnlyInsideFineAlignmentRadius)
{
  const Vec3 home{4.55, -0.38, 1.13};
  const Vec3 waypoint{5.50, -1.70, 1.80};
  const Vec3 approach{4.55, -0.75, 1.80};
  const auto target = returnRouteRawTarget(
    home, {4.55, -0.74, 1.80}, waypoint, approach, true, 0.45,
    0.67, 0.055, 0.02);

  EXPECT_NEAR(target.x, home.x, kTolerance);
  EXPECT_NEAR(target.y, home.y, kTolerance);
  EXPECT_NEAR(target.z, 1.80, kTolerance);
}

TEST(ReturnGuidance, FineAlignmentEntryDependsOnlyOnHorizontalApproach)
{
  const Vec3 home{4.55, -0.38, 1.13};
  EXPECT_TRUE(returnFineAlignmentReady(
      home, {4.55, -0.75, 1.80}, 0.45));
  EXPECT_FALSE(returnFineAlignmentReady(
      home, {4.55, -0.84, 1.20}, 0.45));
}

TEST(ReturnGuidance, TransitWaypointRequiresHorizontalAndVerticalAgreement)
{
  const Vec3 waypoint{5.50, -1.70, 1.80};
  EXPECT_TRUE(returnTransitWaypointReached(
      {5.61, -1.79, 1.88}, waypoint, 0.15, 0.10));
  EXPECT_FALSE(returnTransitWaypointReached(
      {5.66, -1.70, 1.80}, waypoint, 0.15, 0.10));
  EXPECT_FALSE(returnTransitWaypointReached(
      {5.50, -1.70, 1.91}, waypoint, 0.15, 0.10));
}

TEST(ReturnGuidance, ResetsTransitWaypointOnlyForInitialDropToReturnEdge)
{
  EXPECT_TRUE(shouldResetReturnTransitWaypoint(
    FlightPhase::DROP_HOLD, FlightPhase::RETURN));
  EXPECT_FALSE(shouldResetReturnTransitWaypoint(
    FlightPhase::HOLD, FlightPhase::RETURN));
  EXPECT_FALSE(shouldResetReturnTransitWaypoint(
    FlightPhase::RETURN, FlightPhase::RETURN));
}

TEST(ReturnGuidance, FineAlignmentClampsHorizontalAndVerticalVelocity)
{
  const auto velocity = positionAlignmentVelocityEnu(
    {4.0, 0.0, 1.5}, {4.3, -0.4, 1.0}, 1.0, 0.10, 0.05);
  EXPECT_NEAR(std::hypot(velocity.x, velocity.y), 0.10, kTolerance);
  EXPECT_NEAR(velocity.z, -0.05, kTolerance);

  const auto small = positionAlignmentVelocityEnu(
    {4.54, -0.37, 1.18}, {4.55, -0.38, 1.185}, 0.4, 0.10, 0.05);
  EXPECT_NEAR(small.x, 0.004, kTolerance);
  EXPECT_NEAR(small.y, -0.004, kTolerance);
  EXPECT_NEAR(small.z, 0.002, kTolerance);
}

TEST(Px4MessageTimestamp, AdvancesWhenSimulationClockFreezesOrMovesBackward)
{
  EXPECT_EQ(nextMonotonicTimestampMicros(1000U, 0U), 1000U);
  EXPECT_EQ(nextMonotonicTimestampMicros(1000U, 1000U), 1001U);
  EXPECT_EQ(nextMonotonicTimestampMicros(900U, 1001U), 1002U);
  EXPECT_EQ(nextMonotonicTimestampMicros(2000U, 1002U), 2000U);
  EXPECT_EQ(
    nextMonotonicTimestampMicros(
      1U, std::numeric_limits<std::uint64_t>::max()),
    std::numeric_limits<std::uint64_t>::max());
}

TEST(VoxelPlanner, DetoursAroundInflatedObstacle)
{
  PlannerConfig config;
  config.resolution = 0.1;
  config.inflation_radius = 0.2;
  config.horizontal_range = 4.0;
  config.vertical_range = 2.0;
  config.virtual_ceiling = 2.9;
  VoxelPlanner planner(config);

  std::vector<Vec3> wall;
  for (double y = -0.3; y <= 0.3; y += 0.1) {
    for (double z = 0.7; z <= 1.3; z += 0.1) {
      wall.push_back({1.0, y, z});
    }
  }
  planner.setObstacles(wall);

  const auto path = planner.plan({0.0, 0.0, 1.0}, {2.0, 0.0, 1.0});
  ASSERT_GE(path.size(), 3U);
  EXPECT_NEAR(path.front().x, 0.0, kTolerance);
  EXPECT_NEAR(path.back().x, 2.0, kTolerance);
  for (std::size_t index = 1; index < path.size(); ++index) {
    EXPECT_TRUE(planner.collisionFree(path[index - 1], path[index]));
  }
}

TEST(VoxelPlanner, UnreachableGoalHonorsExpansionBudget)
{
  PlannerConfig config;
  config.resolution = 0.1;
  config.inflation_radius = 0.2;
  config.horizontal_range = 5.5;
  config.vertical_range = 4.5;
  config.virtual_ceiling = 2.9;
  config.max_expanded_voxels = 100;
  VoxelPlanner planner(config);

  std::vector<Vec3> sealed_wall;
  for (double y = -5.5; y <= 5.5; y += 0.1) {
    for (double z = 0.0; z <= 2.9; z += 0.1) {
      sealed_wall.push_back({1.0, y, z});
    }
  }
  planner.setObstacles(sealed_wall);

  const auto started = std::chrono::steady_clock::now();
  const auto path = planner.plan({0.0, 0.0, 1.8}, {2.0, 0.0, 1.8});
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_TRUE(path.empty());
  EXPECT_LT(elapsed, std::chrono::milliseconds(250));
}

TEST(VoxelPlanner, FindsWideDetourWithinCompetitionExpansionBudget)
{
  PlannerConfig config;
  config.resolution = 0.1;
  config.inflation_radius = 0.25;
  config.horizontal_range = 5.5;
  config.vertical_range = 4.5;
  config.virtual_ceiling = 2.9;
  config.max_expanded_voxels = 20000;
  VoxelPlanner planner(config);

  std::vector<Vec3> wall;
  for (double y = -2.0; y <= 2.0; y += 0.1) {
    for (double z = 0.0; z <= 2.9; z += 0.1) {
      wall.push_back({2.0, y, z});
    }
  }
  planner.setObstacles(wall);

  const auto path = planner.plan({0.0, 0.0, 1.8}, {4.0, 0.0, 1.8});

  ASSERT_GE(path.size(), 3U);
  for (std::size_t index = 1; index < path.size(); ++index) {
    EXPECT_TRUE(planner.collisionFree(path[index - 1], path[index]));
  }
}

TEST(VoxelPlanner, OccupiedGoalFailsBeforeSearchExpansion)
{
  PlannerConfig config;
  config.resolution = 0.1;
  config.inflation_radius = 0.25;
  config.horizontal_range = 5.5;
  config.vertical_range = 4.5;
  config.max_expanded_voxels = 200000;
  VoxelPlanner planner(config);
  planner.setObstacles({{2.0, 0.0, 1.8}});

  const auto started = std::chrono::steady_clock::now();
  const auto path = planner.plan({0.0, 0.0, 1.8}, {2.0, 0.0, 1.8});
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_TRUE(path.empty());
  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
}

TEST(RollingVoxelMap, RetainsRecentObstaclesAcrossSparseFramesAndExpiresThem)
{
  RollingVoxelMap map(0.1, 1.0);
  map.update({{1.0, 0.0, 1.0}}, 10.0);
  map.update({}, 10.2);

  EXPECT_EQ(map.obstaclesAround({0.0, 0.0, 1.0}, 5.5, 4.5, 10.2).size(), 1U);
  EXPECT_TRUE(map.obstaclesAround({0.0, 0.0, 1.0}, 5.5, 4.5, 11.01).empty());
}

TEST(UniformBsplineTrajectory, HonorsEndpointsAndDynamicLimits)
{
  const auto trajectory = UniformBsplineTrajectory::fromWaypoints(
    {{0.0, 0.0, 1.0}, {1.0, 0.4, 1.2}, {2.0, 0.0, 1.0}}, 0.5, 1.0);
  ASSERT_FALSE(trajectory.empty());
  ASSERT_GT(trajectory.duration(), 0.0);

  const auto start = trajectory.sample(0.0);
  const auto finish = trajectory.sample(trajectory.duration());
  EXPECT_NEAR(start.position.x, 0.0, kTolerance);
  EXPECT_NEAR(start.position.y, 0.0, kTolerance);
  EXPECT_NEAR(finish.position.x, 2.0, kTolerance);
  EXPECT_NEAR(finish.position.y, 0.0, kTolerance);

  for (double time = 0.0; time <= trajectory.duration(); time += 0.02) {
    const auto state = trajectory.sample(time);
    EXPECT_LE(drone_navigation::norm(state.velocity), 0.5 + 1e-5);
    EXPECT_LE(drone_navigation::norm(state.acceleration), 1.0 + 1e-5);
  }
}

TEST(PolylineFallback, StopsAtEveryCollisionFreeCornerAndHonorsDynamicLimits)
{
  const std::vector<Vec3> path{{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}};
  constexpr double kSamplePeriod = 0.05;
  constexpr double kMaximumSpeed = 0.3;
  constexpr double kMaximumAcceleration = 0.6;
  const auto samples = sampleCollisionFreePolyline(
    path, kSamplePeriod, kMaximumSpeed, kMaximumAcceleration);
  ASSERT_GT(samples.size(), 3U);
  EXPECT_NEAR(samples.front().state.position.x, 0.0, kTolerance);
  EXPECT_NEAR(samples.back().state.position.x, 1.0, kTolerance);
  EXPECT_NEAR(samples.back().state.position.y, 1.0, kTolerance);
  bool saw_corner = false;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    EXPECT_LE(
      drone_navigation::norm(samples[index].state.velocity),
      kMaximumSpeed + kTolerance);
    EXPECT_LE(
      drone_navigation::norm(samples[index].state.acceleration),
      kMaximumAcceleration + kTolerance);
    if (drone_navigation::distance(samples[index].state.position, path[1]) <= kTolerance) {
      saw_corner = true;
      EXPECT_NEAR(
        drone_navigation::norm(samples[index].state.velocity), 0.0, kTolerance);
    }
    if (index == 0U) {
      continue;
    }
    EXPECT_GT(
      samples[index].time_from_start_seconds,
      samples[index - 1].time_from_start_seconds);
    const double step_distance = drone_navigation::distance(
      samples[index - 1].state.position, samples[index].state.position);
    const double step_time =
      samples[index].time_from_start_seconds -
      samples[index - 1].time_from_start_seconds;
    EXPECT_LE(step_distance, kMaximumSpeed * step_time + kTolerance);
  }
  EXPECT_TRUE(saw_corner);
  EXPECT_NEAR(
    drone_navigation::norm(samples.back().state.velocity), 0.0, kTolerance);
}

TEST(FlightSupervisor, EnforcesDoorAndDataFreshnessSafetyGates)
{
  FlightSupervisor supervisor;
  SupervisorInputs inputs;
  inputs.mission_requested = true;
  inputs.ground_task_complete = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::PREFLIGHT);

  inputs.side_door_closed = true;
  inputs.px4_ready = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::ARMING);
  EXPECT_TRUE(supervisor.update(inputs).request_arm_offboard);

  inputs.armed = true;
  inputs.offboard = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::TAKEOFF);

  inputs.odometry_age_seconds = 0.31;
  const auto stale = supervisor.update(inputs);
  EXPECT_EQ(stale.phase, FlightPhase::HOLD);
  EXPECT_TRUE(stale.hold_position);

  inputs.odometry_age_seconds = 0.1;
  const auto recovered = supervisor.update(inputs);
  EXPECT_EQ(recovered.phase, FlightPhase::TAKEOFF);
  EXPECT_FALSE(recovered.hold_position);

  inputs.odometry_age_seconds = 0.31;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::HOLD);
  inputs.odometry_age_seconds = 1.1;
  const auto lost = supervisor.update(inputs);
  EXPECT_TRUE(lost.request_land);
}

TEST(FlightSupervisor, RevokesArmRequestWhenReadinessIsLostDuringPrestream)
{
  FlightSupervisor supervisor;
  SupervisorInputs inputs;
  inputs.mission_requested = true;
  inputs.ground_task_complete = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::PREFLIGHT);

  inputs.side_door_closed = true;
  inputs.px4_ready = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::ARMING);
  EXPECT_TRUE(supervisor.update(inputs).request_arm_offboard);

  inputs.px4_ready = false;
  const auto revoked = supervisor.update(inputs);
  EXPECT_EQ(revoked.phase, FlightPhase::PREFLIGHT);
  EXPECT_FALSE(revoked.request_arm_offboard);
  EXPECT_FALSE(revoked.reason.empty());
}

TEST(BoolTokenValue, RejectsMalformedOrAmbiguousPlannerEvidence)
{
  EXPECT_EQ(boolTokenValue("ACTIVE map_ready=true map_age=0.1", "map_ready"), true);
  EXPECT_EQ(boolTokenValue("ACTIVE map_ready=false", "map_ready"), false);
  EXPECT_FALSE(boolTokenValue("ACTIVE not_map_ready=true", "map_ready").has_value());
  EXPECT_FALSE(boolTokenValue("ACTIVE map_ready=trueish", "map_ready").has_value());
  EXPECT_FALSE(
    boolTokenValue("ACTIVE map_ready=true map_ready=false", "map_ready").has_value());
}

TEST(FlightSupervisor, RunsNominalMissionSequence)
{
  FlightSupervisor supervisor;
  SupervisorInputs inputs;
  inputs.mission_requested = true;
  inputs.ground_task_complete = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::PREFLIGHT);
  inputs.side_door_closed = true;
  inputs.px4_ready = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::ARMING);
  inputs.armed = true;
  inputs.offboard = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::TAKEOFF);
  inputs.at_takeoff_pose = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::EGO_TRANSIT);
  inputs.at_search_pose = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::TARGET_SEARCH);
  inputs.target_visible = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::VISUAL_ALIGN);
  inputs.target_aligned = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::DROP_HOLD);
  EXPECT_TRUE(supervisor.update(inputs).command_open_bottom_door);
  inputs.payload_released = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::DROP_HOLD);
  inputs.drop_release_settled = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::RETURN);
  inputs.at_home = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::LAND);
  inputs.landed = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::LAND);
  inputs.armed = false;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::COMPLETE);
}

TEST(FlightSupervisor, ReacquiresTargetAfterVisualLoss)
{
  FlightSupervisor supervisor;
  SupervisorInputs inputs;
  inputs.mission_requested = true;
  inputs.ground_task_complete = true;
  supervisor.update(inputs);
  inputs.side_door_closed = true;
  inputs.px4_ready = true;
  supervisor.update(inputs);
  inputs.armed = true;
  inputs.offboard = true;
  supervisor.update(inputs);
  inputs.at_takeoff_pose = true;
  supervisor.update(inputs);
  inputs.at_search_pose = true;
  supervisor.update(inputs);
  inputs.target_visible = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::VISUAL_ALIGN);
  inputs.target_visible = false;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::TARGET_SEARCH);
}

TEST(FlightSupervisor, LandsOnPx4FailsafeWhileAirborne)
{
  FlightSupervisor supervisor;
  SupervisorInputs inputs;
  inputs.mission_requested = true;
  inputs.ground_task_complete = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::PREFLIGHT);
  inputs.side_door_closed = true;
  inputs.px4_ready = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::ARMING);
  inputs.armed = true;
  inputs.offboard = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::TAKEOFF);

  inputs.px4_failsafe = true;
  const auto decision = supervisor.update(inputs);
  EXPECT_EQ(decision.phase, FlightPhase::LAND);
  EXPECT_TRUE(decision.request_land);
  inputs.px4_failsafe = false;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::LAND);
  EXPECT_TRUE(supervisor.update(inputs).request_land);
}

TEST(ExecutorWatchdog, HoldsThenLandsOnStaleControlIntent)
{
  EXPECT_EQ(
    executorSafetyAction(true, 0.1, 0.1, 0.1, 0.3, 1.0),
    ExecutorSafetyAction::CONTINUE);
  EXPECT_EQ(
    executorSafetyAction(true, 0.1, 0.31, 0.1, 0.3, 1.0),
    ExecutorSafetyAction::HOLD);
  EXPECT_EQ(
    executorSafetyAction(true, 0.1, 1.01, 0.1, 0.3, 1.0),
    ExecutorSafetyAction::LAND);
}

TEST(ExecutorFixedDiagnostic, VerticalOnlyAvoidsConstrainedPositionAndYawWindup)
{
  drone_navigation::TrajectoryState state;
  state.position = {1.0, 2.0, -0.10};
  state.velocity = {0.1, -0.2, -0.01};
  state.acceleration = {0.3, 0.4, -0.02};
  state.yaw = 1.2;

  const auto vertical = fixedDiagnosticControlSetpoint(state, true);
  EXPECT_TRUE(std::isnan(vertical.position[0]));
  EXPECT_TRUE(std::isnan(vertical.position[1]));
  EXPECT_DOUBLE_EQ(vertical.position[2], -0.10);
  EXPECT_TRUE(std::isnan(vertical.velocity[0]));
  EXPECT_TRUE(std::isnan(vertical.velocity[1]));
  EXPECT_DOUBLE_EQ(vertical.velocity[2], -0.01);
  EXPECT_DOUBLE_EQ(vertical.acceleration[0], 0.0);
  EXPECT_DOUBLE_EQ(vertical.acceleration[1], 0.0);
  EXPECT_DOUBLE_EQ(vertical.acceleration[2], -0.02);
  EXPECT_TRUE(std::isnan(vertical.yaw));
  EXPECT_DOUBLE_EQ(vertical.yawspeed, 0.0);

  const auto full = fixedDiagnosticControlSetpoint(state, false);
  EXPECT_DOUBLE_EQ(full.position[0], 1.0);
  EXPECT_DOUBLE_EQ(full.position[1], 2.0);
  EXPECT_DOUBLE_EQ(full.yaw, 1.2);
}

TEST(ExecutorFixedDiagnostic, UsesClearanceHysteresisForHorizontalHandoff)
{
  EXPECT_FALSE(verticalOnlyDiagnosticActive(false, true, true, 0.0, 0.17, 0.16));
  EXPECT_TRUE(verticalOnlyDiagnosticActive(true, true, false, 0.0, 0.17, 0.16));
  EXPECT_TRUE(verticalOnlyDiagnosticActive(true, true, true, 0.16, 0.17, 0.16));
  EXPECT_FALSE(verticalOnlyDiagnosticActive(true, true, true, 0.17, 0.17, 0.16));
  EXPECT_FALSE(verticalOnlyDiagnosticActive(true, true, false, 0.17, 0.17, 0.16));
  EXPECT_FALSE(verticalOnlyDiagnosticActive(true, false, true, 0.165, 0.17, 0.16));
  EXPECT_TRUE(verticalOnlyDiagnosticActive(true, false, true, 0.16, 0.17, 0.16));
  EXPECT_FALSE(verticalOnlyDiagnosticActive(true, false, false, 0.16, 0.17, 0.16));
  EXPECT_TRUE(verticalOnlyDiagnosticActive(
      true, false, true, std::numeric_limits<double>::quiet_NaN(), 0.17, 0.16));
}

TEST(ExecutorFixedDiagnostic, PrefersFreshPhysicalLiftWitness)
{
  EXPECT_DOUBLE_EQ(trustedLiftClearance(0.002, true, 0.006), 0.006);
  EXPECT_DOUBLE_EQ(trustedLiftClearance(0.006, true, 0.002), 0.002);
  EXPECT_DOUBLE_EQ(trustedLiftClearance(0.002, false, 0.006), 0.002);
  EXPECT_DOUBLE_EQ(
    trustedLiftClearance(
      std::numeric_limits<double>::quiet_NaN(), true, 0.006),
    0.006);
  EXPECT_TRUE(std::isnan(trustedLiftClearance(
      std::numeric_limits<double>::quiet_NaN(), false, 0.006)));

  EXPECT_FALSE(verticalOnlyDiagnosticActive(
      true, true, true, trustedLiftClearance(0.002, true, 0.006), 0.005, 0.003));
  EXPECT_TRUE(verticalOnlyDiagnosticActive(
      true, true, true, trustedLiftClearance(0.006, true, 0.002), 0.005, 0.003));
}

TEST(ExecutorFixedDiagnostic, RequiresHorizontalControlBeforePhysicalGuideExit)
{
  EXPECT_TRUE(verticalOnlyHandoffConfigurationSafe(0.040, 0.035, 0.05, 0.005));
  EXPECT_TRUE(verticalOnlyHandoffConfigurationSafe(0.005, 0.003, 0.05, 0.005));
  EXPECT_FALSE(verticalOnlyHandoffConfigurationSafe(0.050, 0.045, 0.05, 0.005));
  EXPECT_FALSE(verticalOnlyHandoffConfigurationSafe(0.035, 0.035, 0.05, 0.005));
}

TEST(ExecutorFixedDiagnostic, BlendsCapturedHorizontalEstimateWithoutAStep)
{
  EXPECT_DOUBLE_EQ(fixedHandoffBlendScale(-0.1, 1.0), 1.0);
  EXPECT_DOUBLE_EQ(fixedHandoffBlendScale(0.0, 1.0), 1.0);
  EXPECT_NEAR(fixedHandoffBlendScale(0.5, 1.0), 0.5, kTolerance);
  EXPECT_DOUBLE_EQ(fixedHandoffBlendScale(1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(fixedHandoffBlendScale(2.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    fixedHandoffBlendScale(std::numeric_limits<double>::quiet_NaN(), 1.0), 0.0);
}

TEST(ExecutorWatchdog, IncludesTrajectoryFreshnessOnlyInTrajectoryMode)
{
  EXPECT_EQ(
    executorSafetyAction(true, 0.1, 0.1, 0.31, 0.3, 1.0),
    ExecutorSafetyAction::HOLD);
  EXPECT_EQ(
    executorSafetyAction(false, 0.1, 0.1, 5.0, 0.3, 1.0),
    ExecutorSafetyAction::CONTINUE);
  EXPECT_EQ(
    executorSafetyAction(true, 1.1, 0.1, 0.1, 0.3, 1.0),
    ExecutorSafetyAction::LAND);
}

TEST(ExecutorWatchdog, StartedTrajectoryRemainsAValidEndpointHold)
{
  EXPECT_TRUE(std::isinf(
    drone_navigation::trajectoryControlSourceAge(false, false, 0.1)));
  EXPECT_DOUBLE_EQ(
    drone_navigation::trajectoryControlSourceAge(true, false, 0.25), 0.25);
  EXPECT_DOUBLE_EQ(
    drone_navigation::trajectoryControlSourceAge(true, true, 42.0), 0.0);
}

TEST(TrajectoryUpdates, PreserveMinimumExecutionWindowWhileActive)
{
  EXPECT_TRUE(drone_navigation::shouldAcceptTrajectoryUpdate(
    false, false, false, 0.0, 1.0));
  EXPECT_TRUE(drone_navigation::shouldAcceptTrajectoryUpdate(
    true, false, false, 0.2, 1.0));
  EXPECT_FALSE(drone_navigation::shouldAcceptTrajectoryUpdate(
    true, true, true, 0.2, 1.0));
  EXPECT_TRUE(drone_navigation::shouldAcceptTrajectoryUpdate(
    true, true, true, 1.0, 1.0));
}

TEST(TrajectoryUpdates, PreservesCurrentTrajectoryUntilItsEndpoint)
{
  EXPECT_DOUBLE_EQ(
    drone_navigation::trajectoryReplacementDelay(4.0, 21.435605657),
    21.435605657);
  EXPECT_DOUBLE_EQ(
    drone_navigation::trajectoryReplacementDelay(4.0, 2.0), 4.0);
  EXPECT_THROW(
    drone_navigation::trajectoryReplacementDelay(-1.0, 2.0),
    std::invalid_argument);
}

TEST(TrajectoryUpdates, ReturnSafetyPathCanPreemptAnAltitudeSag)
{
  EXPECT_TRUE(trajectoryMinimumAltitudeImproves(1.18, 1.72, 0.10));
  EXPECT_FALSE(trajectoryMinimumAltitudeImproves(1.70, 1.72, 0.10));
  EXPECT_FALSE(trajectoryMinimumAltitudeImproves(1.70, 1.60, 0.0));
}

TEST(TrajectoryUpdates, PlannerRecoveryCanReplaceAPathWithTheSameEndpoint)
{
  EXPECT_TRUE(plannerRecoveryAllowsTrajectoryReplacement(
    "NO_PATH start_clear=true goal_clear=false",
    "ACTIVE_POLYLINE_FALLBACK trajectory_id=42"));
  EXPECT_TRUE(plannerRecoveryAllowsTrajectoryReplacement(
    "NO_PATH", "ACTIVE trajectory_id=43"));
  EXPECT_FALSE(plannerRecoveryAllowsTrajectoryReplacement(
    "ACTIVE trajectory_id=41", "ACTIVE_POLYLINE_FALLBACK trajectory_id=42"));
}

TEST(TrajectoryUpdates, ForcedPhaseChangeWaitsForNewEndpoint)
{
  EXPECT_TRUE(forcedTrajectoryEndpointChanged(
      false, {}, {4.55, -0.38, 1.8}, 0.05));
  EXPECT_FALSE(forcedTrajectoryEndpointChanged(
      true, {4.55, -0.38, 1.8}, {4.56, -0.37, 1.81}, 0.05));
  EXPECT_TRUE(forcedTrajectoryEndpointChanged(
      true, {4.55, -0.38, 1.8}, {5.5, -3.5, 1.8}, 0.05));
}

TEST(ExecutorLanding, DisarmsOnlyAfterConfirmedGroundDelay)
{
  using drone_navigation::ExecutorFlightState;
  EXPECT_FALSE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::ACTIVE, true, true, true, true, 3.0, 3.0, 2.0));
  EXPECT_FALSE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::LAND_LATCHED, true, false, true, true, 3.0, 3.0, 2.0));
  EXPECT_FALSE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::LAND_LATCHED, true, true, true, true, 3.0, 1.9, 2.0));
  EXPECT_TRUE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::LAND_LATCHED, true, true, true, true, 3.0, 2.0, 2.0));
  EXPECT_FALSE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::LAND_LATCHED, false, true, true, true, 3.0, 3.0, 2.0));
  EXPECT_FALSE(drone_navigation::shouldRequestGroundDisarm(
    ExecutorFlightState::LAND_LATCHED, true, true, true, false, 3.0, 3.0, 2.0));
}

TEST(ExecutorLanding, ForceDisarmDiagnosticRequiresTerminalAutoLand)
{
  using drone_navigation::forceDisarmDiagnosticAllowed;
  EXPECT_TRUE(forceDisarmDiagnosticAllowed(
    true, ExecutorFlightState::LAND_LATCHED, true, true));
  EXPECT_FALSE(forceDisarmDiagnosticAllowed(
    false, ExecutorFlightState::LAND_LATCHED, true, true));
  EXPECT_FALSE(forceDisarmDiagnosticAllowed(
    true, ExecutorFlightState::ACTIVE, true, true));
  EXPECT_FALSE(forceDisarmDiagnosticAllowed(
    true, ExecutorFlightState::LAND_LATCHED, false, true));
  EXPECT_FALSE(forceDisarmDiagnosticAllowed(
    true, ExecutorFlightState::LAND_LATCHED, true, false));
}

TEST(OperatorLandingLatch, ForceDisarmDiagnosticBypassesLatchedLandProjection)
{
  EXPECT_TRUE(drone_navigation::forceDisarmBypassesLandLatch(
    true, true, "FORCE_DISARM"));
  EXPECT_FALSE(drone_navigation::forceDisarmBypassesLandLatch(
    false, true, "FORCE_DISARM"));
  EXPECT_FALSE(drone_navigation::forceDisarmBypassesLandLatch(
    true, true, "LAND"));
  EXPECT_FALSE(drone_navigation::forceDisarmBypassesLandLatch(
    true, false, "FORCE_DISARM"));
}

TEST(OperatorArmGate, AllowsGroundArmAndAlreadyActiveOffboardTracking)
{
  EXPECT_TRUE(drone_navigation::operatorArmRequestAllowed(
    true, true, true, true, true, true, false, false));
  EXPECT_FALSE(drone_navigation::operatorArmRequestAllowed(
    true, true, true, false, true, true, false, false));
  EXPECT_TRUE(drone_navigation::operatorArmRequestAllowed(
    true, true, true, false, true, false, true, true));
  EXPECT_TRUE(drone_navigation::operatorArmRequestAllowed(
    true, true, true, false, true, true, true, true));
  EXPECT_FALSE(drone_navigation::operatorArmRequestAllowed(
    true, true, true, true, true, false, true, false));
  EXPECT_FALSE(drone_navigation::operatorArmRequestAllowed(
    true, false, true, true, true, true, false, false));
  EXPECT_FALSE(drone_navigation::operatorArmRequestAllowed(
    false, true, true, true, true, true, false, false));
}

TEST(OperatorArmGate, RequiresPx4TiltAgreementWithSimulatorTruth)
{
  const double tolerance = 1.5 * kPi / 180.0;
  EXPECT_TRUE(drone_navigation::prearmAttitudeAgreementAllowed(
    0.0, 0.0, 1.4 * kPi / 180.0, -1.4 * kPi / 180.0, tolerance));
  EXPECT_FALSE(drone_navigation::prearmAttitudeAgreementAllowed(
    0.0, 0.0, 1.6 * kPi / 180.0, 0.0, tolerance));
  EXPECT_TRUE(drone_navigation::prearmAttitudeAgreementAllowed(
    kPi - 0.01, 0.0, -kPi + 0.01, 0.0, 0.03));
  EXPECT_FALSE(drone_navigation::prearmAttitudeAgreementAllowed(
    0.0, 0.0, 0.0, 0.0, -0.1));
}

TEST(PrearmPoseGate, RequiresCalibratedSpawnSpeedAndTiltEnvelope)
{
  drone_navigation::PrearmPoseLimits limits;
  limits.expected_position = {4.55, -0.38, 1.13};
  limits.position_tolerance = 0.02;
  limits.max_speed = 0.05;
  limits.max_tilt_radians = 3.0 * kPi / 180.0;

  drone_navigation::PrearmPoseSample sample;
  sample.position = {4.5513, -0.3819, 1.1299};
  sample.velocity = {0.01, 0.0, 0.0};
  sample.roll_radians = 0.05 * kPi / 180.0;
  sample.pitch_radians = -0.03 * kPi / 180.0;
  EXPECT_TRUE(drone_navigation::prearmPoseAllowed(sample, limits));

  sample.position = {5.1876, -0.3052, 0.2630};
  EXPECT_FALSE(drone_navigation::prearmPoseAllowed(sample, limits));

  sample.position = {4.5513, -0.3819, 1.1299};
  sample.velocity = {0.051, 0.0, 0.0};
  EXPECT_FALSE(drone_navigation::prearmPoseAllowed(sample, limits));

  sample.velocity = {};
  sample.roll_radians = 3.01 * kPi / 180.0;
  EXPECT_FALSE(drone_navigation::prearmPoseAllowed(sample, limits));
}

TEST(PlannerMapGate, RequiresFreshSuccessfulMapUpdate)
{
  EXPECT_TRUE(drone_navigation::freshPlannerMapReady(true, true, 0.20, 0.60));
  EXPECT_FALSE(drone_navigation::freshPlannerMapReady(false, true, 0.20, 0.60));
  EXPECT_FALSE(drone_navigation::freshPlannerMapReady(true, false, 0.20, 0.60));
  EXPECT_FALSE(drone_navigation::freshPlannerMapReady(true, true, 0.61, 0.60));
  EXPECT_FALSE(drone_navigation::freshPlannerMapReady(true, true, 0.20, -0.01));
}

TEST(FixedSetpointGate, RequiresExplicitEnablementAndFreshTarget)
{
  EXPECT_TRUE(fixedSetpointReady(true, true, 0.2, 0.6));
  EXPECT_FALSE(fixedSetpointReady(false, true, 0.2, 0.6));
  EXPECT_FALSE(fixedSetpointReady(true, false, 0.2, 0.6));
  EXPECT_FALSE(fixedSetpointReady(true, true, 0.61, 0.6));
  EXPECT_FALSE(fixedSetpointReady(true, true, -0.1, 0.6));
}

TEST(Px4DiscreteState, UsesCachedStateOnlyWhileContinuousTransportIsAlive)
{
  EXPECT_TRUE(drone_navigation::px4DiscreteStateUsable(true, 0.2, 0.6));
  EXPECT_FALSE(drone_navigation::px4DiscreteStateUsable(false, 0.2, 0.6));
  EXPECT_FALSE(drone_navigation::px4DiscreteStateUsable(true, 0.61, 0.6));
}

TEST(CargoDoorState, IgnoresUnrelatedStatusAndUpdatesOnlyExplicitDoorEvents)
{
  EXPECT_TRUE(drone_navigation::updateSideDoorClosed(
    true, "payload_locked=True prearm_support=True"));
  EXPECT_TRUE(drone_navigation::updateSideDoorClosed(false, "left_closed"));
  EXPECT_FALSE(drone_navigation::updateSideDoorClosed(true, "left_opened"));
}

TEST(ExecutorLifecycle, LandLatchRejectsHoldAndPreventsRearming)
{
  ExecutorLifecycle lifecycle;
  ExecutorLifecycleInputs inputs;
  inputs.landed = false;
  inputs.requested_mode = ExecutorRequestedMode::ARM_TRAJECTORY;
  EXPECT_EQ(lifecycle.update(inputs).state, ExecutorFlightState::PRESTREAM);

  inputs.prestream_complete = true;
  inputs.offboard = true;
  inputs.armed = true;
  EXPECT_EQ(lifecycle.update(inputs).state, ExecutorFlightState::ACTIVE);

  inputs.requested_mode = ExecutorRequestedMode::LAND;
  auto landing = lifecycle.update(inputs);
  EXPECT_EQ(landing.state, ExecutorFlightState::LAND_LATCHED);
  EXPECT_TRUE(landing.request_land);

  inputs.requested_mode = ExecutorRequestedMode::HOLD;
  inputs.auto_land = true;
  inputs.offboard = false;
  inputs.armed = false;
  inputs.landed = true;
  inputs.landed_known = true;
  const auto completed = lifecycle.update(inputs);
  EXPECT_EQ(completed.state, ExecutorFlightState::COMPLETE);
  EXPECT_FALSE(completed.stream_offboard);
  EXPECT_FALSE(completed.request_offboard);
  EXPECT_FALSE(completed.request_arm);

  inputs.auto_land = false;
  inputs.offboard = true;
  const auto stale_hold = lifecycle.update(inputs);
  EXPECT_EQ(stale_hold.state, ExecutorFlightState::COMPLETE);
  EXPECT_FALSE(stale_hold.request_arm);
}

TEST(ExecutorLifecycle, ResetRequiresLandedAndDisarmed)
{
  ExecutorLifecycle lifecycle;
  ExecutorLifecycleInputs inputs;
  inputs.landed = false;
  inputs.requested_mode = ExecutorRequestedMode::ARM_TRAJECTORY;
  lifecycle.update(inputs);
  inputs.offboard = true;
  inputs.armed = true;
  inputs.prestream_complete = true;
  lifecycle.update(inputs);
  inputs.requested_mode = ExecutorRequestedMode::LAND;
  lifecycle.update(inputs);

  inputs.requested_mode = ExecutorRequestedMode::RESET;
  EXPECT_EQ(lifecycle.update(inputs).state, ExecutorFlightState::LAND_LATCHED);

  inputs.armed = false;
  inputs.landed = true;
  inputs.landed_known = true;
  EXPECT_EQ(lifecycle.update(inputs).state, ExecutorFlightState::COMPLETE);
  const auto reset = lifecycle.update(inputs);
  EXPECT_EQ(reset.state, ExecutorFlightState::DISABLED);
  EXPECT_TRUE(reset.request_loiter);
}

TEST(ExecutorLifecycle, GroundLandIsNoOpAndResetCanRecoverDisabledState)
{
  EXPECT_EQ(
    reduceExecutorRequest(
      ExecutorRequestedMode::DISABLED,
      ExecutorRequestedMode::LAND,
      ExecutorFlightState::DISABLED),
    ExecutorRequestedMode::DISABLED);
  EXPECT_EQ(
    reduceExecutorRequest(
      ExecutorRequestedMode::LAND,
      ExecutorRequestedMode::RESET,
      ExecutorFlightState::DISABLED),
    ExecutorRequestedMode::RESET);
}

TEST(ExecutorLifecycle, OnlyExplicitArmRequestCanLeaveDisabled)
{
  ExecutorLifecycle lifecycle;
  ExecutorLifecycleInputs inputs;
  inputs.landed = true;
  inputs.requested_mode = ExecutorRequestedMode::TRAJECTORY;
  auto decision = lifecycle.update(inputs);
  EXPECT_EQ(decision.state, ExecutorFlightState::DISABLED);
  EXPECT_FALSE(decision.request_arm);

  inputs.requested_mode = ExecutorRequestedMode::HOLD;
  decision = lifecycle.update(inputs);
  EXPECT_EQ(decision.state, ExecutorFlightState::DISABLED);
  EXPECT_FALSE(decision.stream_offboard);

  inputs.requested_mode = ExecutorRequestedMode::ARM_TRAJECTORY;
  decision = lifecycle.update(inputs);
  EXPECT_EQ(decision.state, ExecutorFlightState::PRESTREAM);
  EXPECT_FALSE(decision.request_arm);

  inputs.prestream_complete = true;
  decision = lifecycle.update(inputs);
  EXPECT_TRUE(decision.request_offboard);
  inputs.offboard = true;
  decision = lifecycle.update(inputs);
  EXPECT_TRUE(decision.request_arm);
}

TEST(ExecutorLifecycle, AcceptedPreflightSurvivesOffboardReadyTransition)
{
  EXPECT_TRUE(drone_navigation::armCommandAllowed(true, true, false, 1.0));
  EXPECT_FALSE(drone_navigation::armCommandAllowed(true, false, false, 1.0));
  EXPECT_FALSE(drone_navigation::armCommandAllowed(true, true, true, 1.0));
  EXPECT_FALSE(drone_navigation::armCommandAllowed(true, true, false, 0.99));
  EXPECT_FALSE(drone_navigation::armCommandAllowed(false, true, false, 10.0));
}

TEST(ExecutorLifecycle, UnknownLandingStateCannotCompleteLanding)
{
  ExecutorLifecycle lifecycle;
  ExecutorLifecycleInputs inputs;
  inputs.requested_mode = ExecutorRequestedMode::ARM_TRAJECTORY;
  inputs.landed = false;
  lifecycle.update(inputs);
  inputs.prestream_complete = true;
  inputs.offboard = true;
  inputs.armed = true;
  lifecycle.update(inputs);
  inputs.requested_mode = ExecutorRequestedMode::LAND;
  lifecycle.update(inputs);

  inputs.armed = false;
  inputs.landed = true;
  inputs.landed_known = false;
  EXPECT_EQ(lifecycle.update(inputs).state, ExecutorFlightState::LAND_LATCHED);
}

TEST(ExecutorLifecycle, OffboardModeDoesNotBypassFullPrestream)
{
  ExecutorLifecycle lifecycle;
  ExecutorLifecycleInputs inputs;
  inputs.requested_mode = ExecutorRequestedMode::ARM_TRAJECTORY;
  inputs.landed = true;
  lifecycle.update(inputs);

  inputs.offboard = true;
  inputs.prestream_complete = false;
  EXPECT_FALSE(lifecycle.update(inputs).request_arm);

  inputs.prestream_complete = true;
  EXPECT_TRUE(lifecycle.update(inputs).request_arm);
}

TEST(ExecutorLifecycle, LandingRequestIsStickyUntilCompleted)
{
  for (const auto incoming : {
      ExecutorRequestedMode::HOLD,
      ExecutorRequestedMode::TRAJECTORY,
      ExecutorRequestedMode::ARM_TRAJECTORY,
      ExecutorRequestedMode::RESET,
      ExecutorRequestedMode::DISABLED})
  {
    EXPECT_EQ(
      reduceExecutorRequest(
        ExecutorRequestedMode::LAND, incoming, ExecutorFlightState::ACTIVE),
      ExecutorRequestedMode::LAND);
    EXPECT_EQ(
      reduceExecutorRequest(
        ExecutorRequestedMode::LAND, incoming, ExecutorFlightState::LAND_LATCHED),
      ExecutorRequestedMode::LAND);
  }
  EXPECT_EQ(
    reduceExecutorRequest(
      ExecutorRequestedMode::LAND, ExecutorRequestedMode::RESET,
      ExecutorFlightState::COMPLETE),
    ExecutorRequestedMode::RESET);
}

TEST(ExecutorLifecycle, FailsafeLandingStillRequestsPx4Land)
{
  drone_navigation::ExecutorLifecycle lifecycle;
  drone_navigation::ExecutorLifecycleInputs inputs;
  inputs.requested_mode = drone_navigation::ExecutorRequestedMode::ARM_TRAJECTORY;
  EXPECT_EQ(lifecycle.update(inputs).state, drone_navigation::ExecutorFlightState::PRESTREAM);
  inputs.armed = true;
  inputs.offboard = true;
  EXPECT_EQ(lifecycle.update(inputs).state, drone_navigation::ExecutorFlightState::ACTIVE);
  inputs.failsafe = true;
  const auto decision = lifecycle.update(inputs);
  EXPECT_EQ(decision.state, drone_navigation::ExecutorFlightState::LAND_LATCHED);
  EXPECT_FALSE(decision.stream_offboard);
  EXPECT_TRUE(decision.request_land);
}
