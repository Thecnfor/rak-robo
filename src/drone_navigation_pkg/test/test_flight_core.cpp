// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <cmath>
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
using drone_navigation::fixedSetpointReady;
using drone_navigation::verticalOnlyDiagnosticActive;
using drone_navigation::verticalOnlyHandoffConfigurationSafe;

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
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::RETURN);
  inputs.at_home = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::LAND);
  inputs.landed = true;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::LAND);
  inputs.armed = false;
  EXPECT_EQ(supervisor.update(inputs).phase, FlightPhase::COMPLETE);
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

TEST(ExecutorFixedDiagnostic, HandsPositionControlOverOnlyAboveGuideWithHysteresis)
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

TEST(ExecutorFixedDiagnostic, RequiresHorizontalControlBeforePhysicalGuideExit)
{
  EXPECT_TRUE(verticalOnlyHandoffConfigurationSafe(0.04, 0.03, 0.05, 0.005));
  EXPECT_FALSE(verticalOnlyHandoffConfigurationSafe(0.08, 0.06, 0.05, 0.005));
  EXPECT_FALSE(verticalOnlyHandoffConfigurationSafe(0.05, 0.04, 0.05, 0.005));
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
