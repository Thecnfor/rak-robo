// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"
#include "drone_navigation_pkg/msg/trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace drone_navigation
{
namespace navigation_message = drone_navigation_pkg::msg;

class TrajectoryExecutorNode : public rclcpp::Node
{
public:
  TrajectoryExecutorNode()
  : Node("trajectory_executor")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    prestream_seconds_ = declare_parameter<double>("offboard_prestream_seconds", 1.0);
    hold_timeout_ = declare_parameter<double>("odometry_hold_timeout", 0.3);
    land_timeout_ = declare_parameter<double>("data_land_timeout", 1.0);
    minimum_replan_execution_seconds_ = declare_parameter<double>(
      "minimum_replan_execution_seconds", 1.0);
    ground_disarm_delay_seconds_ = declare_parameter<double>(
      "ground_disarm_delay_seconds", 2.0);
    allow_fixed_setpoint_diagnostic_ = declare_parameter<bool>(
      "allow_fixed_setpoint_diagnostic", false);
    allow_force_disarm_diagnostic_ = declare_parameter<bool>(
      "allow_force_disarm_diagnostic", false);
    fixed_vertical_only_diagnostic_ = declare_parameter<bool>(
      "fixed_vertical_only_diagnostic", false);
    fixed_vertical_release_clearance_ = declare_parameter<double>(
      "fixed_vertical_release_clearance", 0.03);
    fixed_vertical_reengage_clearance_ = declare_parameter<double>(
      "fixed_vertical_reengage_clearance", 0.02);
    fixed_vertical_guide_height_ = declare_parameter<double>(
      "fixed_vertical_guide_height", 0.05);
    fixed_vertical_minimum_handoff_lead_ = declare_parameter<double>(
      "fixed_vertical_minimum_handoff_lead", 0.005);
    fixed_vertical_guide_radius_ = declare_parameter<double>(
      "fixed_vertical_guide_radius", 0.004);
    fixed_setpoint_timeout_ = declare_parameter<double>(
      "fixed_setpoint_timeout", 0.6);
    if (hold_timeout_ < 0.0 || land_timeout_ < hold_timeout_) {
      throw std::runtime_error("executor watchdog timeouts must be non-negative and ordered");
    }
    if (minimum_replan_execution_seconds_ < 0.0) {
      throw std::runtime_error("minimum replan execution time must be non-negative");
    }
    if (ground_disarm_delay_seconds_ < 0.0) {
      throw std::runtime_error("ground disarm delay must be non-negative");
    }
    if (fixed_setpoint_timeout_ <= 0.0) {
      throw std::runtime_error("fixed_setpoint_timeout must be positive");
    }
    if (fixed_vertical_only_diagnostic_ && !allow_fixed_setpoint_diagnostic_) {
      throw std::runtime_error(
              "fixed_vertical_only_diagnostic requires allow_fixed_setpoint_diagnostic");
    }
    if (!verticalOnlyHandoffConfigurationSafe(
        fixed_vertical_release_clearance_,
        fixed_vertical_reengage_clearance_,
        fixed_vertical_guide_height_,
        fixed_vertical_minimum_handoff_lead_))
    {
      throw std::runtime_error(
              "fixed vertical control must hand over before the physical guide exit");
    }
    if (fixed_vertical_guide_radius_ <= 0.0) {
      throw std::runtime_error("fixed_vertical_guide_radius must be positive");
    }
    fixed_vertical_only_active_ = fixed_vertical_only_diagnostic_;
    const auto origin = declare_parameter<std::vector<double>>(
      "px4_map_origin", {4.55, -0.38, 1.13});
    if (origin.size() != 3) {
      throw std::runtime_error("px4_map_origin must contain [x, y, z]");
    }
    map_origin_ = {origin[0], origin[1], origin[2]};
    const auto reliable_qos = rclcpp::QoS(10).reliable().durability_volatile();
    offboard_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", reliable_qos);
    setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", reliable_qos);
    command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", reliable_qos);
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/executor_state", rclcpp::QoS(10).transient_local());

    trajectory_subscription_ = create_subscription<navigation_message::Trajectory>(
      "/drone/navigation/trajectory", rclcpp::QoS(1).transient_local(),
      std::bind(&TrajectoryExecutorNode::onTrajectory, this, std::placeholders::_1));
    mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/drone/navigation/control_mode", rclcpp::QoS(10).transient_local(),
      std::bind(&TrajectoryExecutorNode::onMode, this, std::placeholders::_1));
    visual_velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone/navigation/visual_velocity", rclcpp::QoS(10),
      std::bind(&TrajectoryExecutorNode::onVisualVelocity, this, std::placeholders::_1));
    const auto fixed_setpoint_topic = declare_parameter<std::string>(
      "fixed_setpoint_topic", "/drone/navigation/fixed_setpoint");
    fixed_setpoint_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      fixed_setpoint_topic, rclcpp::QoS(1).transient_local(),
      std::bind(&TrajectoryExecutorNode::onFixedSetpoint, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20),
      std::bind(&TrajectoryExecutorNode::onOdometry, this, std::placeholders::_1));
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos,
      std::bind(&TrajectoryExecutorNode::onVehicleStatus, this, std::placeholders::_1));
    land_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&TrajectoryExecutorNode::onLandDetected, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&TrajectoryExecutorNode::tick, this));
  }

private:
  enum class Mode {DISABLED, TRAJECTORY, FIXED, HOLD, VISUAL, LAND};
  using SteadyTime = std::chrono::steady_clock::time_point;

  static double durationSeconds(const builtin_interfaces::msg::Duration & duration)
  {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  std::uint64_t timestampMicros() const
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void publishState(const std::string & text)
  {
    std_msgs::msg::String state;
    state.data = text;
    state_publisher_->publish(state);
  }

  void onTrajectory(const navigation_message::Trajectory::SharedPtr message)
  {
    if (message->points.empty()) {
      publishState("REJECTED empty_trajectory");
      return;
    }
    last_trajectory_ = steadyNow();
    const double accepted_trajectory_age = trajectory_started_ ?
      std::max(0.0, (now() - trajectory_start_).seconds()) :
      std::numeric_limits<double>::infinity();
    if (!shouldAcceptTrajectoryUpdate(
        trajectory_started_, armed_, offboard_, accepted_trajectory_age,
        minimum_replan_execution_seconds_))
    {
      return;
    }
    trajectory_ = *message;
    have_trajectory_ = true;
    trajectory_started_ = armed_ && offboard_;
    if (trajectory_started_) {
      trajectory_start_ = now();
    }
    publishState("TRAJECTORY_ACCEPTED id=" + std::to_string(message->trajectory_id));
  }

  void onMode(const std_msgs::msg::String::SharedPtr message)
  {
    const std::string requested = message->data;
    if (requested == "FORCE_DISARM") {
      if (!forceDisarmDiagnosticAllowed(
          allow_force_disarm_diagnostic_, lifecycle_.state(), armed_, auto_land_))
      {
        publishState("REJECTED force_disarm_gate");
        return;
      }
      force_disarm_requested_ = true;
      last_control_intent_ = steadyNow();
      publishState("FORCE_DISARM_ACCEPTED");
      return;
    }
    Mode new_mode = Mode::DISABLED;
    ExecutorRequestedMode new_request = ExecutorRequestedMode::DISABLED;
    bool fixed_arm_request = false;
    if (requested == "ARM_OFFBOARD" || requested == "ARM_FIXED") {
      const bool fixed_request = requested == "ARM_FIXED";
      fixed_arm_request = fixed_request;
      if (fixed_request && !allow_fixed_setpoint_diagnostic_) {
        publishState("REJECTED fixed_setpoint_diagnostic_disabled");
        return;
      }
      if (lifecycle_.state() == ExecutorFlightState::DISABLED) {
        const bool control_source_ready = fixed_request ?
          fixedSetpointReady(
            allow_fixed_setpoint_diagnostic_, have_fixed_setpoint_,
            ageOrInfinity(last_fixed_setpoint_), fixed_setpoint_timeout_) :
          (have_trajectory_ && ageOrInfinity(last_trajectory_) <= hold_timeout_);
        const bool fresh_inputs = have_odometry_ && control_source_ready &&
          ageOrInfinity(last_odometry_) <= hold_timeout_ &&
          px4DiscreteStateUsable(have_status_, ageOrInfinity(last_odometry_), hold_timeout_);
        if (!fresh_inputs || !px4_ready_ || failsafe_ || !landed_known_ || !landed_ || armed_) {
          publishState("REJECTED arm_preflight_gate");
          return;
        }
      }
      new_mode = fixed_request ? Mode::FIXED : Mode::TRAJECTORY;
      new_request = ExecutorRequestedMode::ARM_TRAJECTORY;
    } else if (requested == "FIXED") {
      if (!fixedSetpointReady(
          allow_fixed_setpoint_diagnostic_, have_fixed_setpoint_,
          ageOrInfinity(last_fixed_setpoint_), fixed_setpoint_timeout_))
      {
        publishState("REJECTED fixed_setpoint_gate");
        return;
      }
      new_mode = Mode::FIXED;
      new_request = ExecutorRequestedMode::TRAJECTORY;
    } else if (requested == "TRAJECTORY" || requested == "TAKEOFF" || requested == "RETURN") {
      new_mode = Mode::TRAJECTORY;
      new_request = ExecutorRequestedMode::TRAJECTORY;
    } else if (requested == "HOLD" || requested == "TARGET_SEARCH" || requested == "DROP_HOLD") {
      new_mode = Mode::HOLD;
      new_request = ExecutorRequestedMode::HOLD;
    } else if (requested == "VISUAL") {
      new_mode = Mode::VISUAL;
      new_request = ExecutorRequestedMode::VISUAL;
    } else if (requested == "LAND") {
      new_mode = Mode::LAND;
      new_request = ExecutorRequestedMode::LAND;
    } else if (requested == "ABORT") {
      new_mode = Mode::LAND;
      new_request = ExecutorRequestedMode::LAND;
    } else if (requested == "RESET") {
      new_mode = Mode::DISABLED;
      new_request = ExecutorRequestedMode::RESET;
    } else if (requested == "DISABLED") {
      new_mode = Mode::DISABLED;
      new_request = ExecutorRequestedMode::DISABLED;
    } else {
      publishState("REJECTED unknown_mode=" + requested);
      return;
    }

    last_control_intent_ = steadyNow();

    const auto reduced_request = reduceExecutorRequest(
      requested_mode_, new_request, lifecycle_.state());
    if (reduced_request != new_request) {
      publishState("IGNORED terminal_landing_latch mode=" + requested);
      return;
    }
    new_request = reduced_request;

    if (new_mode != mode_) {
      if (new_mode == Mode::HOLD && have_odometry_) {
        hold_position_ = current_position_;
      }
      if (new_request == ExecutorRequestedMode::ARM_TRAJECTORY) {
        prestream_started_ = steadyNow();
        if (fixed_arm_request) {
          fixed_vertical_only_active_ = fixed_vertical_only_diagnostic_;
          fixed_diagnostic_yaw_ = current_yaw_;
        }
      }
      mode_ = new_mode;
      publishState("MODE " + requested);
    }
    if (new_request == ExecutorRequestedMode::RESET) {
      have_trajectory_ = false;
      trajectory_started_ = false;
      have_fixed_setpoint_ = false;
      fixed_vertical_only_active_ = fixed_vertical_only_diagnostic_;
    }
    requested_mode_ = new_request;
  }

  void onVisualVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    visual_velocity_ = {
      message->twist.linear.x, message->twist.linear.y, message->twist.linear.z};
    last_visual_velocity_ = steadyNow();
  }

  void onFixedSetpoint(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!allow_fixed_setpoint_diagnostic_) {
      publishState("REJECTED fixed_setpoint_diagnostic_disabled");
      return;
    }
    const auto & position = message->pose.position;
    const auto & orientation = message->pose.orientation;
    const double orientation_norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    if (message->header.frame_id != map_frame_ ||
      !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation_norm_squared) ||
      std::abs(orientation_norm_squared - 1.0) > 0.01)
    {
      publishState("REJECTED invalid_fixed_setpoint");
      return;
    }
    fixed_position_ = {position.x, position.y, position.z};
    const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
    fixed_yaw_ = std::atan2(sin_yaw, cos_yaw);
    have_fixed_setpoint_ = true;
    last_fixed_setpoint_ = steadyNow();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_position_ = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    const auto & orientation = message->pose.pose.orientation;
    const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
    current_yaw_ = std::atan2(sin_yaw, cos_yaw);
    if (!have_odometry_) {
      hold_position_ = current_position_;
    }
    have_odometry_ = true;
    last_odometry_ = steadyNow();
  }

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    auto_land_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
    auto_loiter_ =
      message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
    px4_ready_ = message->pre_flight_checks_pass;
    failsafe_ = message->failsafe;
    have_status_ = true;
    if (have_trajectory_ && armed_ && offboard_ && !trajectory_started_) {
      trajectory_start_ = now();
      trajectory_started_ = true;
    }
  }

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr message)
  {
    landed_known_ = true;
    if (message->landed && !landed_) {
      landed_since_ = steadyNow();
    } else if (!message->landed) {
      landed_since_ = {};
    }
    landed_ = message->landed;
  }

  void tick()
  {
    const auto lifecycle_state = lifecycle_.state();
    const bool watchdog_active = lifecycle_state == ExecutorFlightState::PRESTREAM ||
      lifecycle_state == ExecutorFlightState::ACTIVE ||
      lifecycle_state == ExecutorFlightState::HOLD;
    if (!have_odometry_) {
      if (armed_ || offboard_ || watchdog_active) {
        requested_mode_ = ExecutorRequestedMode::LAND;
        publishState("WAITING_FOR_ODOMETRY");
      }
    } else if (watchdog_active) {
      const bool trajectory_expected =
        (mode_ == Mode::TRAJECTORY || mode_ == Mode::FIXED) && armed_ && offboard_;
      const double control_source_age = mode_ == Mode::FIXED ?
        ageOrInfinity(last_fixed_setpoint_) : ageOrInfinity(last_trajectory_);
      const auto safety_action = executorSafetyAction(
        trajectory_expected,
        ageOrInfinity(last_odometry_),
        ageOrInfinity(last_control_intent_),
        control_source_age,
        hold_timeout_,
        land_timeout_);
      if (safety_action == ExecutorSafetyAction::LAND) {
        requested_mode_ = ExecutorRequestedMode::LAND;
        mode_ = Mode::LAND;
        publishState("WATCHDOG stale_input_land");
      } else if (safety_action == ExecutorSafetyAction::HOLD && mode_ != Mode::HOLD) {
        hold_position_ = current_position_;
        requested_mode_ = ExecutorRequestedMode::HOLD;
        mode_ = Mode::HOLD;
        publishState("WATCHDOG stale_input_hold");
      }
    }

    ExecutorLifecycleInputs inputs;
    inputs.requested_mode = requested_mode_;
    inputs.prestream_complete = ageOrInfinity(prestream_started_) >= prestream_seconds_;
    inputs.armed = armed_;
    inputs.offboard = offboard_;
    inputs.auto_land = auto_land_;
    inputs.auto_loiter = auto_loiter_;
    inputs.landed_known = landed_known_;
    inputs.landed = landed_;
    inputs.failsafe = failsafe_;
    const auto decision = lifecycle_.update(inputs);

    if (decision.state == ExecutorFlightState::LAND_LATCHED) {
      if (land_latched_since_.time_since_epoch().count() == 0) {
        land_latched_since_ = steadyNow();
      }
    } else {
      land_latched_since_ = {};
    }

    if (decision.stream_offboard && have_odometry_) {
      publishOffboardControlMode(mode_ == Mode::VISUAL);
      publishSetpoint();
    }
    if (decision.request_offboard && px4_ready_ &&
      ageOrInfinity(last_mode_command_) >= 1.0)
    {
        publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      last_mode_command_ = steadyNow();
    }
    if (decision.request_arm && px4_ready_ && !failsafe_ &&
      ageOrInfinity(last_arm_command_) >= 1.0)
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      last_arm_command_ = steadyNow();
    }
    if (decision.request_land && ageOrInfinity(last_land_command_) >= 1.0) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      last_land_command_ = steadyNow();
      publishState("LAND_COMMAND_SENT");
    }
    const bool landed_after_latch =
      landed_since_.time_since_epoch().count() != 0 &&
      land_latched_since_.time_since_epoch().count() != 0 &&
      landed_since_ >= land_latched_since_;
    if (shouldRequestGroundDisarm(
        decision.state, armed_, auto_land_, landed_, landed_after_latch,
        ageOrInfinity(landed_since_),
        ageOrInfinity(land_latched_since_), ground_disarm_delay_seconds_) &&
      ageOrInfinity(last_disarm_command_) >= 1.0)
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
      last_disarm_command_ = steadyNow();
      publishState("GROUND_DISARM_COMMAND_SENT");
    }
    if (!armed_) {
      force_disarm_requested_ = false;
    } else if (force_disarm_requested_ &&
      forceDisarmDiagnosticAllowed(
        allow_force_disarm_diagnostic_, decision.state, armed_, auto_land_) &&
      ageOrInfinity(last_disarm_command_) >= 1.0)
    {
      // PX4 requires the force-disarm magic in param2 when its land detector
      // has not recognized an already crashed, physically stationary vehicle.
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        0.0F, 21196.0F);
      last_disarm_command_ = steadyNow();
      publishState("FORCE_DISARM_COMMAND_SENT");
    }
    if (decision.request_loiter && ageOrInfinity(last_loiter_command_) >= 1.0) {
      // PX4 custom main mode AUTO=4, AUTO sub-mode LOITER=3.
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 4.0F, 3.0F);
      last_loiter_command_ = steadyNow();
      publishState("RESET_LOITER_COMMAND_SENT");
    }
    publishLifecycleState(decision.state);
  }

  static SteadyTime steadyNow()
  {
    return std::chrono::steady_clock::now();
  }

  static double ageOrInfinity(const SteadyTime & stamp)
  {
    if (stamp.time_since_epoch().count() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(steadyNow() - stamp).count();
  }

  void publishLifecycleState(ExecutorFlightState state)
  {
    // Publish the current state as a heartbeat. Diagnostic events share this
    // transient-local topic, so a late joiner must not mistake an old event
    // for the lifecycle state.
    std::string lifecycle;
    switch (state) {
      case ExecutorFlightState::DISABLED: lifecycle = "DISABLED"; break;
      case ExecutorFlightState::PRESTREAM: lifecycle = "PRESTREAM"; break;
      case ExecutorFlightState::ACTIVE: lifecycle = "ACTIVE"; break;
      case ExecutorFlightState::HOLD: lifecycle = "HOLD"; break;
      case ExecutorFlightState::LAND_LATCHED: lifecycle = "LAND_LATCHED"; break;
      case ExecutorFlightState::COMPLETE: lifecycle = "COMPLETE"; break;
    }
    publishState(
      "LIFECYCLE " + lifecycle + " fixed_setpoint_enabled=" +
      (allow_fixed_setpoint_diagnostic_ ? "true" : "false") +
      " force_disarm_enabled=" +
      (allow_force_disarm_diagnostic_ ? "true" : "false") +
      " fixed_vertical_only=" +
      (fixed_vertical_only_diagnostic_ ? "true" : "false") +
      " fixed_vertical_active=" +
      (fixed_vertical_only_active_ ? "true" : "false"));
  }

  void publishOffboardControlMode(bool velocity_control)
  {
    px4_msgs::msg::OffboardControlMode message;
    message.timestamp = timestampMicros();
    message.position = !velocity_control;
    message.velocity = velocity_control;
    message.acceleration = false;
    message.attitude = false;
    message.body_rate = false;
    message.thrust_and_torque = false;
    message.direct_actuator = false;
    offboard_publisher_->publish(message);
  }

  void publishSetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint setpoint;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    setpoint.timestamp = timestampMicros();
    setpoint.position = {nan, nan, nan};
    setpoint.velocity = {nan, nan, nan};
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = nan;
    setpoint.yawspeed = nan;

    if (mode_ == Mode::VISUAL) {
      Vec3 velocity{};
      if (ageOrInfinity(last_visual_velocity_) <= 0.3) {
        velocity = visual_velocity_;
      }
      const Vec3 ned_velocity = enuToNed(velocity);
      setpoint.velocity = {
        static_cast<float>(ned_velocity.x),
        static_cast<float>(ned_velocity.y),
        static_cast<float>(ned_velocity.z)};
      setpoint.yaw = 0.0F;
    } else {
      auto state = holdState();
      if (mode_ == Mode::FIXED && have_fixed_setpoint_) {
        state = fixedState();
      } else if (mode_ == Mode::TRAJECTORY && have_trajectory_ && trajectory_started_) {
        state = trajectoryStateAt((now() - trajectory_start_).seconds());
      }
      TrajectoryState state_ned;
      state_ned.position = enuToNed(state.position);
      state_ned.velocity = enuToNed(state.velocity);
      state_ned.acceleration = enuToNed(state.acceleration);
      state_ned.yaw = yawEnuToNed(state.yaw);
      const bool vertical_capable_mode = mode_ == Mode::FIXED || mode_ == Mode::HOLD;
      if (vertical_capable_mode) {
        const bool previous_vertical_only = fixed_vertical_only_active_;
        const double horizontal_from_origin = std::hypot(
          current_position_.x - map_origin_.x,
          current_position_.y - map_origin_.y);
        fixed_vertical_only_active_ = verticalOnlyDiagnosticActive(
          fixed_vertical_only_diagnostic_,
          fixed_vertical_only_active_,
          horizontal_from_origin <= fixed_vertical_guide_radius_,
          current_position_.z - map_origin_.z,
          fixed_vertical_release_clearance_,
          fixed_vertical_reengage_clearance_);
        if (previous_vertical_only && !fixed_vertical_only_active_) {
          // The guide prevents real yaw motion but the estimator can drift.
          // Capture the live estimate only when the guide is cleared so full
          // heading control starts with zero error instead of releasing an
          // accumulated yaw command.
          fixed_diagnostic_yaw_ = current_yaw_;
        }
        if (fixed_vertical_only_active_ != previous_vertical_only) {
          publishState(
            std::string("FIXED_CONTROL vertical_only=") +
            (fixed_vertical_only_active_ ? "true" : "false") +
            " clearance=" + std::to_string(current_position_.z - map_origin_.z));
        }
      }
      const bool vertical_only_diagnostic = vertical_capable_mode &&
        fixed_vertical_only_active_;
      if (fixed_vertical_only_diagnostic_ && vertical_capable_mode) {
        state_ned.yaw = yawEnuToNed(fixed_diagnostic_yaw_);
      }
      const auto control_setpoint = fixedDiagnosticControlSetpoint(
        state_ned,
        vertical_only_diagnostic);
      setpoint.position = {
        static_cast<float>(control_setpoint.position[0]),
        static_cast<float>(control_setpoint.position[1]),
        static_cast<float>(control_setpoint.position[2])};
      setpoint.velocity = {
        static_cast<float>(control_setpoint.velocity[0]),
        static_cast<float>(control_setpoint.velocity[1]),
        static_cast<float>(control_setpoint.velocity[2])};
      setpoint.acceleration = {
        static_cast<float>(control_setpoint.acceleration[0]),
        static_cast<float>(control_setpoint.acceleration[1]),
        static_cast<float>(control_setpoint.acceleration[2])};
      setpoint.yaw = static_cast<float>(control_setpoint.yaw);
      setpoint.yawspeed = static_cast<float>(control_setpoint.yawspeed);
    }
    setpoint_publisher_->publish(setpoint);
  }

  TrajectoryState holdState() const
  {
    TrajectoryState state;
    // PX4 local coordinates start at zero. Remove the configured map origin in the adapter path.
    state.position = hold_position_ - map_origin_;
    return state;
  }

  TrajectoryState fixedState() const
  {
    TrajectoryState state;
    state.position = fixed_position_ - map_origin_;
    state.yaw = fixed_yaw_;
    return state;
  }

  TrajectoryState trajectoryStateAt(double seconds) const
  {
    TrajectoryState output;
    const auto & points = trajectory_.points;
    const auto iterator = std::lower_bound(
      points.begin(), points.end(), seconds,
      [](const navigation_message::TrajectoryPoint & point, double value) {
        return durationSeconds(point.time_from_start) < value;
      });
    const auto & point = iterator == points.end() ? points.back() : *iterator;
    output.position = {
      point.position.x - map_origin_.x,
      point.position.y - map_origin_.y,
      point.position.z - map_origin_.z};
    output.velocity = {point.velocity.x, point.velocity.y, point.velocity.z};
    output.acceleration = {
      point.acceleration.x, point.acceleration.y, point.acceleration.z};
    output.yaw = point.yaw;
    return output;
  }

  void publishVehicleCommand(
    std::uint32_t command, float param1 = 0.0F, float param2 = 0.0F,
    float param3 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand message;
    message.timestamp = timestampMicros();
    message.param1 = param1;
    message.param2 = param2;
    message.param3 = param3;
    message.command = command;
    message.target_system = 1;
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.confirmation = 0;
    message.from_external = true;
    command_publisher_->publish(message);
  }

  double prestream_seconds_{1.0};
  std::string map_frame_;
  double hold_timeout_{0.3};
  double land_timeout_{1.0};
  double minimum_replan_execution_seconds_{1.0};
  double ground_disarm_delay_seconds_{2.0};
  double fixed_setpoint_timeout_{0.6};
  double fixed_vertical_release_clearance_{0.03};
  double fixed_vertical_reengage_clearance_{0.02};
  double fixed_vertical_guide_height_{0.05};
  double fixed_vertical_minimum_handoff_lead_{0.005};
  double fixed_vertical_guide_radius_{0.004};
  bool allow_fixed_setpoint_diagnostic_{false};
  bool allow_force_disarm_diagnostic_{false};
  bool fixed_vertical_only_diagnostic_{false};
  bool fixed_vertical_only_active_{false};
  bool force_disarm_requested_{false};
  Vec3 map_origin_;
  Mode mode_{Mode::DISABLED};
  ExecutorRequestedMode requested_mode_{ExecutorRequestedMode::DISABLED};
  ExecutorLifecycle lifecycle_;
  navigation_message::Trajectory trajectory_;
  bool have_trajectory_{false};
  bool have_fixed_setpoint_{false};
  bool trajectory_started_{false};
  bool have_odometry_{false};
  bool armed_{false};
  bool offboard_{false};
  bool auto_land_{false};
  bool auto_loiter_{false};
  bool landed_known_{false};
  bool landed_{false};
  bool px4_ready_{false};
  bool have_status_{false};
  bool failsafe_{false};
  Vec3 current_position_;
  double current_yaw_{0.0};
  double fixed_diagnostic_yaw_{0.0};
  Vec3 hold_position_;
  Vec3 visual_velocity_;
  Vec3 fixed_position_;
  double fixed_yaw_{0.0};
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  SteadyTime prestream_started_{};
  SteadyTime last_mode_command_{};
  SteadyTime last_arm_command_{};
  SteadyTime last_disarm_command_{};
  SteadyTime last_land_command_{};
  SteadyTime last_loiter_command_{};
  SteadyTime last_visual_velocity_{};
  SteadyTime last_fixed_setpoint_{};
  SteadyTime last_odometry_{};
  SteadyTime last_control_intent_{};
  SteadyTime last_trajectory_{};
  SteadyTime landed_since_{};
  SteadyTime land_latched_since_{};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<navigation_message::Trajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr visual_velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fixed_setpoint_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_navigation::TrajectoryExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
