// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace drone_navigation
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class FlightSupervisorNode : public rclcpp::Node
{
public:
  FlightSupervisorNode()
  : Node("flight_supervisor")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    mission_autostart_ = declare_parameter<bool>("mission_autostart", false);
    allow_fixed_setpoint_diagnostic_ = declare_parameter<bool>(
      "allow_fixed_setpoint_diagnostic", false);
    allow_force_disarm_diagnostic_ = declare_parameter<bool>(
      "allow_force_disarm_diagnostic", false);
    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.8);
    hold_timeout_ = declare_parameter<double>("odometry_hold_timeout", 0.3);
    land_timeout_ = declare_parameter<double>("data_land_timeout", 1.0);
    planner_map_timeout_ = declare_parameter<double>("planner_map_timeout", 0.6);
    const double prearm_attitude_tolerance_degrees = declare_parameter<double>(
      "prearm_attitude_tolerance_deg", 0.5);
    if (planner_map_timeout_ <= 0.0) {
      throw std::runtime_error("planner_map_timeout must be positive");
    }
    if (prearm_attitude_tolerance_degrees < 0.0) {
      throw std::runtime_error("prearm attitude tolerance must be non-negative");
    }
    prearm_attitude_tolerance_radians_ =
      prearm_attitude_tolerance_degrees * kPi / 180.0;
    pose_tolerance_ = declare_parameter<double>("pose_tolerance", 0.20);
    visual_offset_threshold_ = declare_parameter<double>("visual_offset_threshold", 0.04);
    visual_alignment_seconds_ = declare_parameter<double>("visual_alignment_seconds", 0.8);
    visual_target_loss_grace_seconds_ = declare_parameter<double>(
      "visual_target_loss_grace_seconds", 0.6);
    visual_target_control_timeout_seconds_ = declare_parameter<double>(
      "visual_target_control_timeout_seconds", 0.4);
    visual_kp_ = declare_parameter<double>("visual_kp", 0.25);
    visual_max_velocity_ = declare_parameter<double>("visual_max_velocity", 0.20);
    drop_min_height_ = declare_parameter<double>("drop_min_height", 1.6);
    drop_max_height_ = declare_parameter<double>("drop_max_height", 2.0);
    payload_release_settle_seconds_ = declare_parameter<double>(
      "payload_release_settle_seconds", 2.0);
    return_transit_clearance_ = declare_parameter<double>(
      "return_transit_clearance", 0.67);
    return_approach_clearance_ = declare_parameter<double>(
      "return_approach_clearance", 0.055);
    return_descent_radius_ = declare_parameter<double>(
      "return_descent_radius", 0.20);
    return_truth_xy_gain_ = declare_parameter<double>("return_truth_xy_gain", 2.0);
    return_horizontal_tolerance_ = declare_parameter<double>(
      "return_horizontal_tolerance", 0.02);
    return_max_speed_ = declare_parameter<double>("return_max_speed", 0.05);
    return_fine_radius_ = declare_parameter<double>("return_fine_radius", 0.25);
    return_fine_kp_ = declare_parameter<double>("return_fine_kp", 0.4);
    return_fine_max_velocity_ = declare_parameter<double>(
      "return_fine_max_velocity", 0.08);
    return_fine_max_vertical_velocity_ = declare_parameter<double>(
      "return_fine_max_vertical_velocity", 0.05);
    return_goal_update_seconds_ = declare_parameter<double>(
      "return_goal_update_seconds", 0.5);
    const auto prearm_spawn = declare_parameter<std::vector<double>>(
      "prearm_spawn_position", {4.55, -0.38, 1.13});
    if (prearm_spawn.size() != 3) {
      throw std::runtime_error("prearm_spawn_position must contain [x, y, z]");
    }
    prearm_limits_.expected_position = {
      prearm_spawn[0], prearm_spawn[1], prearm_spawn[2]};
    prearm_limits_.position_tolerance = declare_parameter<double>(
      "prearm_position_tolerance", 0.004);
    prearm_limits_.max_speed = declare_parameter<double>("prearm_max_speed", 0.05);
    prearm_limits_.max_tilt_radians = declare_parameter<double>(
      "prearm_max_tilt_deg", 3.0) * kPi / 180.0;
    if (prearm_limits_.position_tolerance < 0.0 || prearm_limits_.max_speed < 0.0 ||
      prearm_limits_.max_tilt_radians < 0.0)
    {
      throw std::runtime_error("prearm pose limits must be non-negative");
    }
    if (visual_target_loss_grace_seconds_ < 0.0 ||
      visual_target_control_timeout_seconds_ < 0.0 ||
      visual_target_control_timeout_seconds_ > visual_target_loss_grace_seconds_ ||
      drop_min_height_ < 0.0 || drop_min_height_ > drop_max_height_)
    {
      throw std::runtime_error(
              "visual freshness windows and drop height window must be non-negative and ordered");
    }
    if (payload_release_settle_seconds_ < 0.0 ||
      return_transit_clearance_ <= return_approach_clearance_ ||
      return_approach_clearance_ <= 0.0 || return_descent_radius_ <= 0.0 ||
      return_truth_xy_gain_ <= 0.0 || return_truth_xy_gain_ > 3.0 ||
      return_horizontal_tolerance_ <= 0.0 || return_max_speed_ <= 0.0 ||
      return_goal_update_seconds_ <= 0.0 || return_fine_radius_ <= 0.0 ||
      return_fine_kp_ <= 0.0 || return_fine_max_velocity_ <= 0.0 ||
      return_fine_max_vertical_velocity_ <= 0.0)
    {
      throw std::runtime_error("drop and return guidance limits are invalid");
    }
    const auto search = declare_parameter<std::vector<double>>(
      "drop_search_pose", {0.0, 0.0, 1.8});
    if (search.size() != 3) {
      throw std::runtime_error("drop_search_pose must contain [x, y, z]");
    }
    search_pose_ = {search[0], search[1], search[2]};
    const auto return_transit_waypoint = declare_parameter<std::vector<double>>(
      "return_transit_waypoint", {5.5, -1.7, 1.8});
    if (return_transit_waypoint.size() != 3) {
      throw std::runtime_error("return_transit_waypoint must contain [x, y, z]");
    }
    return_transit_waypoint_ = {
      return_transit_waypoint[0],
      return_transit_waypoint[1],
      return_transit_waypoint[2]};
    const auto return_approach_pose = declare_parameter<std::vector<double>>(
      "return_approach_pose", {4.55, -0.75, 1.8});
    if (return_approach_pose.size() != 3) {
      throw std::runtime_error("return_approach_pose must contain [x, y, z]");
    }
    return_approach_pose_ = {
      return_approach_pose[0],
      return_approach_pose[1],
      return_approach_pose[2]};
    return_transit_waypoint_radius_ = declare_parameter<double>(
      "return_transit_waypoint_radius", 0.15);
    return_transit_waypoint_vertical_tolerance_ = declare_parameter<double>(
      "return_transit_waypoint_vertical_tolerance", 0.15);
    if (return_transit_waypoint_radius_ <= 0.0 ||
      return_transit_waypoint_vertical_tolerance_ <= 0.0)
    {
      throw std::runtime_error("return transit waypoint tolerances must be positive");
    }
    if (!returnFineAlignmentReady(
        prearm_limits_.expected_position, return_approach_pose_, return_fine_radius_))
    {
      throw std::runtime_error(
              "return_approach_pose must be inside return_fine_radius from home");
    }

    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/state", rclcpp::QoS(10).transient_local());
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone/navigation/goal", rclcpp::QoS(1).transient_local());
    const auto fixed_setpoint_topic = declare_parameter<std::string>(
      "fixed_setpoint_topic", "/drone/navigation/fixed_setpoint");
    fixed_setpoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      fixed_setpoint_topic, rclcpp::QoS(1).transient_local());
    control_mode_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/control_mode", rclcpp::QoS(10).transient_local());
    cargo_command_publisher_ = create_publisher<std_msgs::msg::String>(
      "/cargo_bay/command", rclcpp::QoS(10));
    visual_velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/drone/navigation/visual_velocity", rclcpp::QoS(10));

    mission_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone/navigation/mission_request", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {mission_requested_ = message->data;});
    operator_goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/navigation/operator_goal", rclcpp::QoS(1).transient_local(),
      std::bind(&FlightSupervisorNode::onOperatorGoal, this, std::placeholders::_1));
    operator_mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/drone/navigation/operator_mode", rclcpp::QoS(10).transient_local(),
      std::bind(&FlightSupervisorNode::onOperatorMode, this, std::placeholders::_1));
    ground_subscription_ = create_subscription<std_msgs::msg::String>(
      "/arena/ground/state", rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        // Completion is monotonic for one mission.  A still-running ground
        // node may publish an older IDLE sample after the completion edge;
        // that must not revoke permission while the air mission is starting.
        ground_complete_ = ground_complete_ ||
          message->data == "COMPLETE" || message->data == "SUCCESS" ||
          message->data == "GROUND_DONE";
      });
    cargo_subscription_ = create_subscription<std_msgs::msg::String>(
      "/cargo_bay/status", rclcpp::QoS(10),
      std::bind(&FlightSupervisorNode::onCargoStatus, this, std::placeholders::_1));
    px4_subscription_ = create_subscription<std_msgs::msg::String>(
      "/drone/navigation/px4_status", rclcpp::QoS(10).transient_local(),
      std::bind(&FlightSupervisorNode::onPx4Status, this, std::placeholders::_1));
    planner_state_subscription_ = create_subscription<std_msgs::msg::String>(
      "/drone/navigation/planner_state", rclcpp::QoS(10).transient_local(),
      std::bind(&FlightSupervisorNode::onPlannerState, this, std::placeholders::_1));
    landed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone/navigation/landed", rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        have_landed_status_ = true;
        landed_ = message->data;
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20),
      std::bind(&FlightSupervisorNode::onOdometry, this, std::placeholders::_1));
    raw_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone0/state/pose", rclcpp::SensorDataQoS(),
      std::bind(&FlightSupervisorNode::onRawPose, this, std::placeholders::_1));
    raw_twist_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/drone0/state/twist", rclcpp::SensorDataQoS(),
      std::bind(&FlightSupervisorNode::onRawTwist, this, std::placeholders::_1));
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/avoidance/lidar/pointcloud", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
        last_pointcloud_time_ = steadyNow();
      });
    target_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/drone/drop_target_offset", rclcpp::QoS(10),
      std::bind(&FlightSupervisorNode::onTargetOffset, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FlightSupervisorNode::tick, this));
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static bool contains(const std::string & text, const std::string & token)
  {
    return text.find(token) != std::string::npos;
  }

  void onOperatorGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      RCLCPP_WARN(get_logger(), "Rejected operator goal outside frame %s", map_frame_.c_str());
      return;
    }
    operator_goal_ = {
      message->pose.position.x, message->pose.position.y, message->pose.position.z};
    have_operator_goal_ = true;
  }

  void onOperatorMode(const std_msgs::msg::String::SharedPtr message)
  {
    if (message->data == "CLEAR") {
      operator_mode_.reset();
      return;
    }
    const bool fixed_diagnostic_mode =
      message->data == "ARM_FIXED" || message->data == "FIXED";
    if (fixed_diagnostic_mode && !allow_fixed_setpoint_diagnostic_) {
      RCLCPP_WARN(get_logger(), "Rejected disabled fixed-setpoint diagnostic mode");
      return;
    }
    if (message->data == "FORCE_DISARM" && !allow_force_disarm_diagnostic_) {
      RCLCPP_WARN(get_logger(), "Rejected disabled force-disarm diagnostic mode");
      return;
    }
    if (message->data == "ARM_OFFBOARD" || message->data == "ARM_FIXED" ||
      message->data == "TRAJECTORY" || message->data == "FIXED" ||
      message->data == "RETURN" || message->data == "HOLD" ||
      message->data == "VISUAL" || message->data == "LAND" ||
      message->data == "RESET" ||
      message->data == "FORCE_DISARM")
    {
      operator_mode_ = message->data;
      if (message->data == "LAND") {
        operator_land_latched_ = true;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Rejected unknown operator mode %s", message->data.c_str());
    }
  }

  void onCargoStatus(const std_msgs::msg::String::SharedPtr message)
  {
    side_door_closed_ = updateSideDoorClosed(side_door_closed_, message->data);
    if (!payload_released_ && contains(message->data, "payload_released")) {
      payload_released_ = true;
      payload_released_at_ = now();
    }
  }

  void onPx4Status(const std_msgs::msg::String::SharedPtr message)
  {
    px4_ready_ = contains(message->data, "ready=true");
    armed_ = contains(message->data, "armed=true");
    offboard_ = contains(message->data, "offboard=true");
    px4_failsafe_ = contains(message->data, "failsafe=true");
    have_px4_status_ = true;
  }

  void onPlannerState(const std_msgs::msg::String::SharedPtr message)
  {
    planner_map_ready_ = boolTokenValue(message->data, "map_ready").value_or(false);
    have_planner_state_ = true;
    last_planner_state_time_ = steadyNow();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_position_ = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    current_velocity_ = {
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z};
    const auto & orientation = message->pose.pose.orientation;
    const double orientation_norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    have_estimated_attitude_ = std::isfinite(orientation_norm_squared) &&
      std::abs(orientation_norm_squared - 1.0) <= 0.01;
    if (have_estimated_attitude_) {
      const double sin_roll = 2.0 *
        (orientation.w * orientation.x + orientation.y * orientation.z);
      const double cos_roll = 1.0 - 2.0 *
        (orientation.x * orientation.x + orientation.y * orientation.y);
      estimated_roll_radians_ = std::atan2(sin_roll, cos_roll);
      const double sin_pitch = std::clamp(
        2.0 * (orientation.w * orientation.y - orientation.z * orientation.x),
        -1.0, 1.0);
    estimated_pitch_radians_ = std::asin(sin_pitch);
    const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
    estimated_yaw_radians_ = std::atan2(sin_yaw, cos_yaw);
    }
    last_odometry_time_ = steadyNow();
    if (!have_home_ && !armed_) {
      home_ = current_position_;
      have_home_ = true;
    }
  }

  void onRawPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      have_raw_pose_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected prearm raw pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    prearm_sample_.position = {
      message->pose.position.x, message->pose.position.y, message->pose.position.z};
    const auto & orientation = message->pose.orientation;
    const double orientation_norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    if (!std::isfinite(orientation_norm_squared) ||
      std::abs(orientation_norm_squared - 1.0) > 0.01)
    {
      have_raw_pose_ = false;
      return;
    }
    const double sin_roll = 2.0 *
      (orientation.w * orientation.x + orientation.y * orientation.z);
    const double cos_roll = 1.0 - 2.0 *
      (orientation.x * orientation.x + orientation.y * orientation.y);
    prearm_sample_.roll_radians = std::atan2(sin_roll, cos_roll);
    const double sin_pitch = std::clamp(
      2.0 * (orientation.w * orientation.y - orientation.z * orientation.x), -1.0, 1.0);
    prearm_sample_.pitch_radians = std::asin(sin_pitch);
    if (!armed_) {
      raw_home_ = prearm_sample_.position;
      have_raw_home_ = true;
    }
    have_raw_pose_ = true;
    last_raw_pose_time_ = steadyNow();
  }

  void onRawTwist(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    prearm_sample_.velocity = {
      message->twist.linear.x, message->twist.linear.y, message->twist.linear.z};
    have_raw_twist_ = true;
    last_raw_twist_time_ = steadyNow();
  }

  void onTargetOffset(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() < 4 || message->data[2] <= 0.0F) {
      // Invalid render frames do not refresh the last valid observation.
      // tick() expires it after the configured loss-grace interval.
      return;
    }
    target_visible_ = true;
    target_nx_ = message->data[0];
    target_ny_ = message->data[1];
    last_target_time_ = steadyNow();
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

  bool near(const Vec3 & target) const
  {
    return distance(current_position_, target) <= pose_tolerance_;
  }

  void tick()
  {
    const auto previous_phase = supervisor_.phase();
    const double target_age = ageOrInfinity(last_target_time_);
    const bool recent_target = visualTargetRecent(
      target_visible_, target_age, visual_target_loss_grace_seconds_);
    const bool control_target_recent = visualTargetRecent(
      target_visible_, target_age,
      visual_target_control_timeout_seconds_);
    const bool instant_alignment = control_target_recent &&
      std::hypot(target_nx_, target_ny_) <= visual_offset_threshold_ &&
      std::hypot(current_velocity_.x, current_velocity_.y) < 0.05 &&
      current_position_.z >= drop_min_height_ && current_position_.z <= drop_max_height_;
    if (instant_alignment) {
      if (!aligned_since_.has_value()) {
        aligned_since_ = now();
      }
    } else {
      aligned_since_.reset();
    }
    const bool stable_alignment = aligned_since_.has_value() &&
      (now() - *aligned_since_).seconds() >= visual_alignment_seconds_;

    SupervisorInputs inputs;
    inputs.mission_requested = mission_requested_ || mission_autostart_;
    inputs.ground_task_complete = ground_complete_;
    inputs.side_door_closed = side_door_closed_;
    const bool navigation_inputs_ready = have_home_ &&
      ageOrInfinity(last_odometry_time_) <= hold_timeout_ &&
      ageOrInfinity(last_pointcloud_time_) <= hold_timeout_;
    // VehicleStatus is event-driven in PX4, not a heartbeat. Couple the cached
    // discrete state to continuously arriving vehicle odometry instead.
    const bool px4_state_usable = px4DiscreteStateUsable(
      have_px4_status_, ageOrInfinity(last_odometry_time_), hold_timeout_);
    const bool prearm_pose_valid = have_raw_pose_ && have_raw_twist_ &&
      ageOrInfinity(last_raw_pose_time_) <= hold_timeout_ &&
      ageOrInfinity(last_raw_twist_time_) <= hold_timeout_ &&
      prearmPoseAllowed(prearm_sample_, prearm_limits_) &&
      have_estimated_attitude_ && prearmAttitudeAgreementAllowed(
        prearm_sample_.roll_radians, prearm_sample_.pitch_radians,
        estimated_roll_radians_, estimated_pitch_radians_,
        prearm_attitude_tolerance_radians_);
    const bool planner_map_ready = freshPlannerMapReady(
      have_planner_state_, planner_map_ready_,
      ageOrInfinity(last_planner_state_time_), planner_map_timeout_);
    const bool base_px4_ready =
      px4_ready_ && !px4_failsafe_ && px4_state_usable && navigation_inputs_ready;
    // The calibrated support envelope is a ground-arm gate only. Once armed,
    // leaving the support must not interrupt the active Offboard stream.
    inputs.px4_ready = base_px4_ready &&
      (armed_ || (prearm_pose_valid && planner_map_ready));
    inputs.armed = armed_;
    inputs.offboard = offboard_;
    inputs.px4_failsafe = px4_failsafe_;
    inputs.at_takeoff_pose = have_home_ && near({home_.x, home_.y, takeoff_height_});
    inputs.at_search_pose = near(search_pose_);
    inputs.target_visible = recent_target;
    inputs.target_aligned = stable_alignment;
    inputs.payload_released = payload_released_;
    inputs.drop_release_settled = payload_released_at_.has_value() &&
      (now() - *payload_released_at_).seconds() >= payload_release_settle_seconds_;
    const Vec3 raw_return_target = have_raw_home_ ?
      Vec3{raw_home_.x, raw_home_.y, raw_home_.z + return_approach_clearance_} :
      Vec3{};
    const bool raw_return_ready = have_raw_home_ && have_raw_pose_ && have_raw_twist_ &&
      ageOrInfinity(last_raw_pose_time_) <= hold_timeout_ &&
      ageOrInfinity(last_raw_twist_time_) <= hold_timeout_;
    inputs.at_home = raw_return_ready ?
      (std::hypot(
        prearm_sample_.position.x - raw_return_target.x,
        prearm_sample_.position.y - raw_return_target.y) <= return_horizontal_tolerance_ &&
      std::abs(prearm_sample_.position.z - raw_return_target.z) <= pose_tolerance_ &&
      norm(prearm_sample_.velocity) <= return_max_speed_) :
      (have_home_ && near({
        home_.x, home_.y, home_.z + return_approach_clearance_}));
    inputs.landed = have_landed_status_ && landed_;
    inputs.odometry_age_seconds = ageOrInfinity(last_odometry_time_);
    inputs.pointcloud_age_seconds = ageOrInfinity(last_pointcloud_time_);
    inputs.hold_timeout_seconds = hold_timeout_;
    inputs.land_timeout_seconds = land_timeout_;

    // The pure core owns all safety transitions. ROS commands below are projections of its decision.
    auto decision = supervisor_.update(inputs);
    const bool return_fine_active =
      decision.phase == FlightPhase::RETURN && raw_return_ready &&
      returnFineAlignmentReady(
      raw_home_, prearm_sample_.position, return_fine_radius_);
    const bool operator_override = operator_mode_.has_value();
    const double worst_navigation_age = std::max(
      ageOrInfinity(last_odometry_time_), ageOrInfinity(last_pointcloud_time_));
    const bool operator_airborne = operator_override &&
      (armed_ || (have_landed_status_ && !landed_));
    if (operator_airborne &&
      (px4_failsafe_ || worst_navigation_age > land_timeout_))
    {
      operator_land_latched_ = true;
    }
    const bool operator_arm_requested = operator_override &&
      (*operator_mode_ == "ARM_OFFBOARD" || *operator_mode_ == "ARM_FIXED");
    const bool operator_arm_allowed = operator_arm_requested && operatorArmRequestAllowed(
      have_operator_goal_, side_door_closed_, inputs.px4_ready,
      prearm_pose_valid, have_landed_status_, landed_, armed_, offboard_);
    const bool force_disarm_override = operator_override &&
      forceDisarmBypassesLandLatch(
        allow_force_disarm_diagnostic_, operator_land_latched_, *operator_mode_);
    if (!operator_override && decision.command_close_side_door) {
      publishString(cargo_command_publisher_, "left_close");
    }
    if (!operator_override && decision.command_open_bottom_door) {
      publishString(cargo_command_publisher_, "bottom_open");
    }
    if (force_disarm_override) {
      // A diagnostic FORCE_DISARM is only accepted after LAND has latched.
      // Forward it ahead of the ordinary latch projection; the executor still
      // requires terminal AUTO_LAND, armed state, and its explicit opt-in.
      publishOperatorOverride();
    } else if (decision.request_land) {
      publishString(control_mode_publisher_, "LAND");
    } else if (operator_land_latched_) {
      if (operator_override && *operator_mode_ == "RESET" &&
        have_landed_status_ && landed_ && !armed_)
      {
        publishString(control_mode_publisher_, "RESET");
        operator_land_latched_ = false;
      } else {
        publishString(control_mode_publisher_, "LAND");
      }
    } else if (decision.hold_position) {
      publishString(control_mode_publisher_, "HOLD");
    } else if (operator_airborne && worst_navigation_age > hold_timeout_) {
      publishString(control_mode_publisher_, "HOLD");
    } else if (operator_arm_requested && !operator_arm_allowed)
    {
      publishString(control_mode_publisher_, "DISABLED");
    } else if (!operator_override && return_fine_active) {
      publishString(control_mode_publisher_, "RETURN_FINE");
    } else if (operator_override) {
      publishOperatorOverride();
    } else {
      publishControlHeartbeat(decision.phase);
    }

    const auto phase = decision.phase;
    if (!operator_override && phase != previous_phase) {
      onPhaseEntered(previous_phase, phase);
    }
    if (!operator_override && phase == FlightPhase::VISUAL_ALIGN) {
      // Keep the phase latched across brief render misses, but command zero
      // velocity once the shorter control timeout expires.
      publishVisualVelocity(control_target_recent);
    }
    if (!operator_override && phase == FlightPhase::RETURN) {
      if (return_fine_active) {
        publishReturnFineVelocity();
      } else {
        publishReturnGoal(false);
      }
    }

    std_msgs::msg::String state;
    state.data = toString(phase);
    if (!decision.reason.empty()) {
      state.data += " reason=" + decision.reason;
    }
    if (operator_override) {
      state.data += " operator_override=" + *operator_mode_;
    }
    state.data += std::string(" prearm_pose_valid=") +
      (prearm_pose_valid ? "true" : "false");
    state.data += std::string(" prearm_attitude_agreement=") +
      (have_raw_pose_ && have_estimated_attitude_ && prearmAttitudeAgreementAllowed(
        prearm_sample_.roll_radians, prearm_sample_.pitch_radians,
        estimated_roll_radians_, estimated_pitch_radians_,
        prearm_attitude_tolerance_radians_) ? "true" : "false");
    state.data += std::string(" planner_map_ready=") +
      (planner_map_ready ? "true" : "false");
    if (phase == FlightPhase::RETURN) {
      state.data += std::string(" return_waypoint_reached=") +
        (return_transit_waypoint_reached_ ? "true" : "false");
    }
    state_publisher_->publish(state);
  }

  void publishOperatorOverride()
  {
    if (*operator_mode_ == "ARM_OFFBOARD") {
      if (have_operator_goal_) {
        publishGoal(operator_goal_);
        publishString(control_mode_publisher_, "ARM_OFFBOARD");
      } else {
        publishString(control_mode_publisher_, "DISABLED");
      }
      return;
    }
    if (*operator_mode_ == "ARM_FIXED") {
      if (have_operator_goal_ && allow_fixed_setpoint_diagnostic_) {
        publishFixedSetpoint(operator_goal_);
        publishString(control_mode_publisher_, "ARM_FIXED");
      } else {
        publishString(control_mode_publisher_, "DISABLED");
      }
      return;
    }
    if (*operator_mode_ == "FIXED") {
      if (have_operator_goal_ && allow_fixed_setpoint_diagnostic_) {
        publishFixedSetpoint(operator_goal_);
        publishString(control_mode_publisher_, "FIXED");
      } else {
        publishString(control_mode_publisher_, "HOLD");
      }
      return;
    }
    if (*operator_mode_ == "TRAJECTORY") {
      if (have_operator_goal_) {
        publishGoal(operator_goal_);
        publishString(control_mode_publisher_, "TRAJECTORY");
      } else {
        publishString(control_mode_publisher_, "HOLD");
      }
      return;
    }
    if (*operator_mode_ == "RETURN") {
      if (have_home_) {
        publishGoal({home_.x, home_.y, takeoff_height_});
        publishString(control_mode_publisher_, "RETURN");
      } else {
        publishString(control_mode_publisher_, "HOLD");
      }
      return;
    }
    if (*operator_mode_ == "RESET") {
      publishString(control_mode_publisher_, "RESET");
      return;
    }
    publishString(control_mode_publisher_, *operator_mode_);
  }

  void onPhaseEntered(FlightPhase previous_phase, FlightPhase phase)
  {
    switch (phase) {
      case FlightPhase::PREFLIGHT:
        return_descent_latched_ = false;
        publishString(cargo_command_publisher_, "left_close");
        break;
      case FlightPhase::ARMING:
        if (have_home_) {
          publishGoal({home_.x, home_.y, takeoff_height_});
        }
        publishString(control_mode_publisher_, "ARM_OFFBOARD");
        break;
      case FlightPhase::TAKEOFF:
        if (have_home_) {
          publishGoal({home_.x, home_.y, takeoff_height_});
        }
        publishString(control_mode_publisher_, "TAKEOFF");
        break;
      case FlightPhase::EGO_TRANSIT:
        publishGoal(search_pose_);
        publishString(control_mode_publisher_, "TRAJECTORY");
        break;
      case FlightPhase::TARGET_SEARCH:
        publishString(control_mode_publisher_, "TARGET_SEARCH");
        break;
      case FlightPhase::VISUAL_ALIGN:
        publishString(control_mode_publisher_, "VISUAL");
        break;
      case FlightPhase::DROP_HOLD:
        publishString(control_mode_publisher_, "DROP_HOLD");
        publishString(cargo_command_publisher_, "bottom_open");
        break;
      case FlightPhase::RETURN:
        if (shouldResetReturnTransitWaypoint(previous_phase, phase)) {
          return_transit_waypoint_reached_ = false;
          return_descent_latched_ = false;
        }
        publishReturnGoal(true);
        publishString(control_mode_publisher_, "RETURN");
        break;
      case FlightPhase::LAND:
        publishString(control_mode_publisher_, "LAND");
        break;
      case FlightPhase::COMPLETE:
        publishString(control_mode_publisher_, "DISABLED");
        break;
      case FlightPhase::HOLD:
        break;
      case FlightPhase::IDLE:
        break;
    }
  }

  void publishControlHeartbeat(FlightPhase phase)
  {
    switch (phase) {
      case FlightPhase::ARMING:
        publishString(control_mode_publisher_, "ARM_OFFBOARD");
        break;
      case FlightPhase::TAKEOFF:
        publishString(control_mode_publisher_, "TAKEOFF");
        break;
      case FlightPhase::EGO_TRANSIT:
        publishString(control_mode_publisher_, "TRAJECTORY");
        break;
      case FlightPhase::TARGET_SEARCH:
        publishString(control_mode_publisher_, "TARGET_SEARCH");
        break;
      case FlightPhase::VISUAL_ALIGN:
        publishString(control_mode_publisher_, "VISUAL");
        break;
      case FlightPhase::DROP_HOLD:
        publishString(control_mode_publisher_, "DROP_HOLD");
        break;
      case FlightPhase::RETURN:
        publishString(control_mode_publisher_, "RETURN");
        break;
      case FlightPhase::LAND:
        publishString(control_mode_publisher_, "LAND");
        break;
      case FlightPhase::IDLE:
      case FlightPhase::PREFLIGHT:
      case FlightPhase::COMPLETE:
        publishString(control_mode_publisher_, "DISABLED");
        break;
      case FlightPhase::HOLD:
        publishString(control_mode_publisher_, "HOLD");
        break;
    }
  }

  void publishGoal(const Vec3 & point)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = map_frame_;
    goal.pose.position.x = point.x;
    goal.pose.position.y = point.y;
    goal.pose.position.z = point.z;
    goal.pose.orientation.w = 1.0;
    goal_publisher_->publish(goal);
  }

  void publishFixedSetpoint(const Vec3 & point)
  {
    geometry_msgs::msg::PoseStamped setpoint;
    setpoint.header.stamp = now();
    setpoint.header.frame_id = map_frame_;
    setpoint.pose.position.x = point.x;
    setpoint.pose.position.y = point.y;
    setpoint.pose.position.z = point.z;
    setpoint.pose.orientation.w = 1.0;
    fixed_setpoint_publisher_->publish(setpoint);
  }

  void publishVisualVelocity(bool target_recent)
  {
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = map_frame_;
    if (target_recent) {
      const Vec3 correction = visualAlignmentVelocityEnu(
        target_nx_, target_ny_, estimated_yaw_radians_,
        visual_kp_, visual_max_velocity_);
      velocity.twist.linear.x = correction.x;
      velocity.twist.linear.y = correction.y;
    }
    visual_velocity_publisher_->publish(velocity);
  }

  void publishReturnFineVelocity()
  {
    return_descent_latched_ = updateReturnDescentLatch(
      return_descent_latched_, raw_home_, prearm_sample_.position,
      return_descent_radius_);
    const Vec3 staged_target = stagedReturnRawTarget(
      raw_home_, prearm_sample_.position, return_transit_clearance_,
      return_approach_clearance_, return_descent_radius_,
      return_descent_latched_);
    const Vec3 correction = positionAlignmentVelocityEnu(
      prearm_sample_.position, staged_target, return_fine_kp_,
      return_fine_max_velocity_, return_fine_max_vertical_velocity_);
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = map_frame_;
    velocity.twist.linear.x = correction.x;
    velocity.twist.linear.y = correction.y;
    velocity.twist.linear.z = correction.z;
    visual_velocity_publisher_->publish(velocity);
  }

  void publishReturnGoal(bool force)
  {
    if (!have_home_) {
      return;
    }
    const auto current_time = steadyNow();
    if (!force && last_return_goal_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(current_time - last_return_goal_time_).count() <
      return_goal_update_seconds_)
    {
      return;
    }

    const bool raw_return_ready = have_raw_home_ && have_raw_pose_ &&
      ageOrInfinity(last_raw_pose_time_) <= hold_timeout_;
    const Vec3 route_position =
      raw_return_ready ? prearm_sample_.position : current_position_;
    if (!return_transit_waypoint_reached_ &&
      returnTransitWaypointReached(
        route_position, return_transit_waypoint_,
        return_transit_waypoint_radius_,
        return_transit_waypoint_vertical_tolerance_))
    {
      return_transit_waypoint_reached_ = true;
      // The executor normally protects an active trajectory until its end.
      // This explicit route edge grants exactly one replacement for the new
      // home endpoint without weakening ordinary rolling-replan stability.
      publishString(control_mode_publisher_, "RETURN_ROUTE_UPDATE");
    }
    Vec3 target = returnRouteRawTarget(
      home_, current_position_, return_transit_waypoint_, return_approach_pose_,
      return_transit_waypoint_reached_, return_fine_radius_, return_transit_clearance_,
      return_approach_clearance_, return_descent_radius_);
    if (raw_return_ready) {
      const Vec3 raw_target = returnRouteRawTarget(
        raw_home_, prearm_sample_.position, return_transit_waypoint_, return_approach_pose_,
        return_transit_waypoint_reached_, return_fine_radius_, return_transit_clearance_,
        return_approach_clearance_, return_descent_radius_);
      target = rawErrorCorrectedNavigationTarget(
        raw_target, prearm_sample_.position, current_position_, return_truth_xy_gain_);
    }
    publishGoal(target);
    last_return_goal_time_ = current_time;
  }

  static void publishString(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    publisher->publish(message);
  }

  std::string map_frame_;
  bool mission_autostart_{false};
  bool allow_fixed_setpoint_diagnostic_{false};
  bool allow_force_disarm_diagnostic_{false};
  double takeoff_height_{1.8};
  double hold_timeout_{0.3};
  double planner_map_timeout_{0.6};
  double prearm_attitude_tolerance_radians_{0.5 * kPi / 180.0};
  double land_timeout_{1.0};
  double pose_tolerance_{0.2};
  double visual_offset_threshold_{0.04};
  double visual_alignment_seconds_{0.8};
  double visual_target_loss_grace_seconds_{0.6};
  double visual_target_control_timeout_seconds_{0.4};
  double visual_kp_{0.25};
  double visual_max_velocity_{0.2};
  double drop_min_height_{1.6};
  double drop_max_height_{2.0};
  double payload_release_settle_seconds_{2.0};
  double return_transit_clearance_{0.67};
  double return_approach_clearance_{0.055};
  double return_descent_radius_{0.20};
  double return_truth_xy_gain_{2.0};
  double return_horizontal_tolerance_{0.02};
  double return_max_speed_{0.05};
  double return_fine_radius_{0.25};
  double return_fine_kp_{0.4};
  double return_fine_max_velocity_{0.08};
  double return_fine_max_vertical_velocity_{0.05};
  double return_goal_update_seconds_{0.5};
  FlightSupervisor supervisor_;
  Vec3 search_pose_;
  Vec3 return_transit_waypoint_;
  Vec3 return_approach_pose_;
  Vec3 home_;
  Vec3 raw_home_;
  Vec3 current_position_;
  Vec3 current_velocity_;
  PrearmPoseSample prearm_sample_;
  PrearmPoseLimits prearm_limits_;
  Vec3 operator_goal_;
  std::optional<std::string> operator_mode_;
  bool mission_requested_{false};
  bool ground_complete_{false};
  bool side_door_closed_{false};
  bool px4_ready_{false};
  bool have_px4_status_{false};
  bool planner_map_ready_{false};
  bool have_planner_state_{false};
  bool px4_failsafe_{false};
  bool armed_{false};
  bool offboard_{false};
  bool landed_{false};
  bool have_landed_status_{false};
  bool have_home_{false};
  bool have_raw_home_{false};
  bool have_raw_pose_{false};
  bool have_raw_twist_{false};
  bool have_estimated_attitude_{false};
  bool target_visible_{false};
  bool payload_released_{false};
  bool return_transit_waypoint_reached_{false};
  bool return_descent_latched_{false};
  bool have_operator_goal_{false};
  bool operator_land_latched_{false};
  double target_nx_{0.0};
  double target_ny_{0.0};
  double estimated_roll_radians_{0.0};
  double estimated_pitch_radians_{0.0};
  double estimated_yaw_radians_{0.0};
  double return_transit_waypoint_radius_{0.15};
  double return_transit_waypoint_vertical_tolerance_{0.15};
  SteadyTime last_odometry_time_{};
  SteadyTime last_planner_state_time_{};
  SteadyTime last_raw_pose_time_{};
  SteadyTime last_raw_twist_time_{};
  SteadyTime last_pointcloud_time_{};
  SteadyTime last_target_time_{};
  SteadyTime last_return_goal_time_{};
  std::optional<rclcpp::Time> aligned_since_;
  std::optional<rclcpp::Time> payload_released_at_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fixed_setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cargo_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr visual_velocity_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    operator_goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operator_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ground_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cargo_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr px4_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landed_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr raw_twist_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_navigation::FlightSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
