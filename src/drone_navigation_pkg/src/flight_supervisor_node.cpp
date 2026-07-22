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

class FlightSupervisorNode : public rclcpp::Node
{
public:
  FlightSupervisorNode()
  : Node("flight_supervisor")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    mission_autostart_ = declare_parameter<bool>("mission_autostart", false);
    takeoff_height_ = declare_parameter<double>("takeoff_height", 1.8);
    hold_timeout_ = declare_parameter<double>("odometry_hold_timeout", 0.3);
    land_timeout_ = declare_parameter<double>("data_land_timeout", 1.0);
    pose_tolerance_ = declare_parameter<double>("pose_tolerance", 0.20);
    visual_offset_threshold_ = declare_parameter<double>("visual_offset_threshold", 0.04);
    visual_alignment_seconds_ = declare_parameter<double>("visual_alignment_seconds", 0.8);
    visual_kp_ = declare_parameter<double>("visual_kp", 0.25);
    visual_max_velocity_ = declare_parameter<double>("visual_max_velocity", 0.20);
    drop_min_height_ = declare_parameter<double>("drop_min_height", 1.6);
    drop_max_height_ = declare_parameter<double>("drop_max_height", 2.0);
    if (drop_min_height_ < 0.0 || drop_min_height_ > drop_max_height_) {
      throw std::runtime_error("drop height window must be non-negative and ordered");
    }
    const auto search = declare_parameter<std::vector<double>>(
      "drop_search_pose", {0.0, 0.0, 1.8});
    if (search.size() != 3) {
      throw std::runtime_error("drop_search_pose must contain [x, y, z]");
    }
    search_pose_ = {search[0], search[1], search[2]};

    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/state", rclcpp::QoS(10).transient_local());
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone/navigation/goal", rclcpp::QoS(1).transient_local());
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
        ground_complete_ = message->data == "COMPLETE" || message->data == "SUCCESS" ||
        message->data == "GROUND_DONE";
      });
    cargo_subscription_ = create_subscription<std_msgs::msg::String>(
      "/cargo_bay/status", rclcpp::QoS(10),
      std::bind(&FlightSupervisorNode::onCargoStatus, this, std::placeholders::_1));
    px4_subscription_ = create_subscription<std_msgs::msg::String>(
      "/drone/navigation/px4_status", rclcpp::QoS(10).transient_local(),
      std::bind(&FlightSupervisorNode::onPx4Status, this, std::placeholders::_1));
    landed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone/navigation/landed", rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        have_landed_status_ = true;
        landed_ = message->data;
      });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20),
      std::bind(&FlightSupervisorNode::onOdometry, this, std::placeholders::_1));
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
    if (message->data == "ARM_OFFBOARD" || message->data == "TRAJECTORY" ||
      message->data == "RETURN" || message->data == "HOLD" ||
      message->data == "LAND" || message->data == "RESET")
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
    payload_released_ = payload_released_ || contains(message->data, "payload_released");
  }

  void onPx4Status(const std_msgs::msg::String::SharedPtr message)
  {
    px4_ready_ = contains(message->data, "ready=true");
    armed_ = contains(message->data, "armed=true");
    offboard_ = contains(message->data, "offboard=true");
    px4_failsafe_ = contains(message->data, "failsafe=true");
    have_px4_status_ = true;
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
    last_odometry_time_ = steadyNow();
    if (!have_home_ && !armed_) {
      home_ = current_position_;
      have_home_ = true;
    }
  }

  void onTargetOffset(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() < 4 || message->data[2] <= 0.0F) {
      target_visible_ = false;
      aligned_since_.reset();
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
    const bool recent_target = target_visible_ && ageOrInfinity(last_target_time_) <= 0.3;
    const bool instant_alignment = recent_target &&
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
    inputs.px4_ready =
      px4_ready_ && !px4_failsafe_ && px4_state_usable && navigation_inputs_ready;
    inputs.armed = armed_;
    inputs.offboard = offboard_;
    inputs.px4_failsafe = px4_failsafe_;
    inputs.at_takeoff_pose = have_home_ && near({home_.x, home_.y, takeoff_height_});
    inputs.at_search_pose = near(search_pose_);
    inputs.target_visible = recent_target;
    inputs.target_aligned = stable_alignment;
    inputs.payload_released = payload_released_;
    inputs.at_home = have_home_ && near({home_.x, home_.y, takeoff_height_});
    inputs.landed = have_landed_status_ && landed_;
    inputs.odometry_age_seconds = ageOrInfinity(last_odometry_time_);
    inputs.pointcloud_age_seconds = ageOrInfinity(last_pointcloud_time_);
    inputs.hold_timeout_seconds = hold_timeout_;
    inputs.land_timeout_seconds = land_timeout_;

    // The pure core owns all safety transitions. ROS commands below are projections of its decision.
    auto decision = supervisor_.update(inputs);
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
    const bool operator_arm_allowed = operator_override &&
      *operator_mode_ == "ARM_OFFBOARD" && operatorArmRequestAllowed(
      have_operator_goal_, side_door_closed_, inputs.px4_ready,
      have_landed_status_, landed_, armed_, offboard_);
    if (!operator_override && decision.command_close_side_door) {
      publishString(cargo_command_publisher_, "left_close");
    }
    if (!operator_override && decision.command_open_bottom_door) {
      publishString(cargo_command_publisher_, "bottom_open");
    }
    if (decision.request_land) {
      publishString(control_mode_publisher_, "LAND");
    } else if (decision.hold_position) {
      publishString(control_mode_publisher_, "HOLD");
    } else if (operator_land_latched_) {
      if (operator_override && *operator_mode_ == "RESET" &&
        have_landed_status_ && landed_ && !armed_)
      {
        publishString(control_mode_publisher_, "RESET");
        operator_land_latched_ = false;
      } else {
        publishString(control_mode_publisher_, "LAND");
      }
    } else if (operator_airborne && worst_navigation_age > hold_timeout_) {
      publishString(control_mode_publisher_, "HOLD");
    } else if (operator_override && *operator_mode_ == "ARM_OFFBOARD" &&
      !operator_arm_allowed)
    {
      publishString(control_mode_publisher_, "DISABLED");
    } else if (operator_override) {
      publishOperatorOverride();
    } else {
      publishControlHeartbeat(decision.phase);
    }

    const auto phase = decision.phase;
    if (!operator_override && phase != previous_phase) {
      onPhaseEntered(phase);
    }
    if (!operator_override && phase == FlightPhase::VISUAL_ALIGN) {
      publishVisualVelocity(recent_target);
    }

    std_msgs::msg::String state;
    state.data = toString(phase);
    if (!decision.reason.empty()) {
      state.data += " reason=" + decision.reason;
    }
    if (operator_override) {
      state.data += " operator_override=" + *operator_mode_;
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

  void onPhaseEntered(FlightPhase phase)
  {
    switch (phase) {
      case FlightPhase::PREFLIGHT:
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
        if (have_home_) {
          publishGoal({home_.x, home_.y, takeoff_height_});
        }
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

  void publishVisualVelocity(bool target_recent)
  {
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = map_frame_;
    if (target_recent) {
      // Default down-camera mounting: image +y maps to world -x and image +x maps to world -y.
      velocity.twist.linear.x = std::clamp(
        -visual_kp_ * target_ny_, -visual_max_velocity_, visual_max_velocity_);
      velocity.twist.linear.y = std::clamp(
        -visual_kp_ * target_nx_, -visual_max_velocity_, visual_max_velocity_);
    }
    visual_velocity_publisher_->publish(velocity);
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
  double takeoff_height_{1.8};
  double hold_timeout_{0.3};
  double land_timeout_{1.0};
  double pose_tolerance_{0.2};
  double visual_offset_threshold_{0.04};
  double visual_alignment_seconds_{0.8};
  double visual_kp_{0.25};
  double visual_max_velocity_{0.2};
  double drop_min_height_{1.6};
  double drop_max_height_{2.0};
  FlightSupervisor supervisor_;
  Vec3 search_pose_;
  Vec3 home_;
  Vec3 current_position_;
  Vec3 current_velocity_;
  Vec3 operator_goal_;
  std::optional<std::string> operator_mode_;
  bool mission_requested_{false};
  bool ground_complete_{false};
  bool side_door_closed_{false};
  bool px4_ready_{false};
  bool have_px4_status_{false};
  bool px4_failsafe_{false};
  bool armed_{false};
  bool offboard_{false};
  bool landed_{false};
  bool have_landed_status_{false};
  bool have_home_{false};
  bool target_visible_{false};
  bool payload_released_{false};
  bool have_operator_goal_{false};
  bool operator_land_latched_{false};
  double target_nx_{0.0};
  double target_ny_{0.0};
  SteadyTime last_odometry_time_{};
  SteadyTime last_pointcloud_time_{};
  SteadyTime last_target_time_{};
  std::optional<rclcpp::Time> aligned_since_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landed_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
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
