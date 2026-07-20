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

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
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
    prestream_seconds_ = declare_parameter<double>("offboard_prestream_seconds", 1.0);
    hold_timeout_ = declare_parameter<double>("odometry_hold_timeout", 0.3);
    land_timeout_ = declare_parameter<double>("data_land_timeout", 1.0);
    if (hold_timeout_ < 0.0 || land_timeout_ < hold_timeout_) {
      throw std::runtime_error("executor watchdog timeouts must be non-negative and ordered");
    }
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
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20),
      std::bind(&TrajectoryExecutorNode::onOdometry, this, std::placeholders::_1));
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", px4_qos,
      std::bind(&TrajectoryExecutorNode::onVehicleStatus, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&TrajectoryExecutorNode::tick, this));
  }

private:
  enum class Mode {DISABLED, TRAJECTORY, HOLD, VISUAL, LAND};

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
    trajectory_ = *message;
    have_trajectory_ = true;
    last_trajectory_ = now();
    trajectory_started_ = armed_ && offboard_;
    if (trajectory_started_) {
      trajectory_start_ = now();
    }
    publishState("TRAJECTORY_ACCEPTED id=" + std::to_string(message->trajectory_id));
  }

  void onMode(const std_msgs::msg::String::SharedPtr message)
  {
    const std::string requested = message->data;
    Mode new_mode = Mode::DISABLED;
    if (requested == "ARM_OFFBOARD" || requested == "TRAJECTORY" || requested == "TAKEOFF" ||
      requested == "RETURN")
    {
      new_mode = Mode::TRAJECTORY;
    } else if (requested == "HOLD" || requested == "TARGET_SEARCH" || requested == "DROP_HOLD") {
      new_mode = Mode::HOLD;
    } else if (requested == "VISUAL") {
      new_mode = Mode::VISUAL;
    } else if (requested == "LAND") {
      new_mode = Mode::LAND;
    } else if (requested == "ABORT" || requested == "DISABLED") {
      new_mode = Mode::DISABLED;
    } else {
      publishState("REJECTED unknown_mode=" + requested);
      return;
    }

    last_control_intent_ = now();

    if (new_mode != mode_) {
      if (new_mode == Mode::HOLD && have_odometry_) {
        hold_position_ = current_position_;
      }
      if (new_mode == Mode::LAND) {
        land_command_sent_ = false;
      }
      if (mode_ == Mode::DISABLED && new_mode != Mode::DISABLED && new_mode != Mode::LAND) {
        prestream_started_ = now();
        mode_command_sent_ = false;
        arm_command_sent_ = false;
      }
      mode_ = new_mode;
      publishState("MODE " + requested);
    }
  }

  void onVisualVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    visual_velocity_ = {
      message->twist.linear.x, message->twist.linear.y, message->twist.linear.z};
    last_visual_velocity_ = now();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_position_ = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    if (!have_odometry_) {
      hold_position_ = current_position_;
    }
    have_odometry_ = true;
    last_odometry_ = now();
  }

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    offboard_ = message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    failsafe_ = message->failsafe;
    if (have_trajectory_ && armed_ && offboard_ && !trajectory_started_) {
      trajectory_start_ = now();
      trajectory_started_ = true;
    }
  }

  void tick()
  {
    if (mode_ == Mode::DISABLED) {
      return;
    }
    if (mode_ == Mode::LAND) {
      if (!land_command_sent_) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        land_command_sent_ = true;
        publishState("LAND_COMMAND_SENT");
      }
      return;
    }
    if (!have_odometry_) {
      if (armed_ || offboard_) {
        enterLand("WATCHDOG no_odometry_while_active");
      }
      publishState("WAITING_FOR_ODOMETRY");
      return;
    }
    if (failsafe_) {
      publishState("PX4_FAILSAFE");
      return;
    }

    const bool trajectory_expected =
      mode_ == Mode::TRAJECTORY && armed_ && offboard_;
    const auto safety_action = executorSafetyAction(
      trajectory_expected,
      ageOrInfinity(last_odometry_),
      ageOrInfinity(last_control_intent_),
      ageOrInfinity(last_trajectory_),
      hold_timeout_,
      land_timeout_);
    if (safety_action == ExecutorSafetyAction::LAND) {
      enterLand("WATCHDOG stale_input_land");
      return;
    }
    if (safety_action == ExecutorSafetyAction::HOLD) {
      if (mode_ != Mode::HOLD) {
        hold_position_ = current_position_;
        mode_ = Mode::HOLD;
        publishState("WATCHDOG stale_input_hold");
      }
    }

    publishOffboardControlMode(mode_ == Mode::VISUAL);
    publishSetpoint();

    const double streaming_seconds = (now() - prestream_started_).seconds();
    if (streaming_seconds >= prestream_seconds_ && !offboard_) {
      if (!mode_command_sent_ || (now() - last_mode_command_).seconds() >= 1.0) {
        publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        mode_command_sent_ = true;
        last_mode_command_ = now();
      }
      return;
    }
    if (offboard_ && !armed_ &&
      (!arm_command_sent_ || (now() - last_arm_command_).seconds() >= 1.0))
    {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      arm_command_sent_ = true;
      last_arm_command_ = now();
    }
  }

  double ageOrInfinity(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - stamp).seconds();
  }

  void enterLand(const std::string & reason)
  {
    mode_ = Mode::LAND;
    if (!land_command_sent_) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      land_command_sent_ = true;
    }
    publishState(reason);
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
      if ((now() - last_visual_velocity_).seconds() <= 0.3) {
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
      if (mode_ == Mode::TRAJECTORY && have_trajectory_ && trajectory_started_) {
        state = trajectoryStateAt((now() - trajectory_start_).seconds());
      }
      const Vec3 ned_position = enuToNed(state.position);
      const Vec3 ned_velocity = enuToNed(state.velocity);
      const Vec3 ned_acceleration = enuToNed(state.acceleration);
      setpoint.position = {
        static_cast<float>(ned_position.x),
        static_cast<float>(ned_position.y),
        static_cast<float>(ned_position.z)};
      setpoint.velocity = {
        static_cast<float>(ned_velocity.x),
        static_cast<float>(ned_velocity.y),
        static_cast<float>(ned_velocity.z)};
      setpoint.acceleration = {
        static_cast<float>(ned_acceleration.x),
        static_cast<float>(ned_acceleration.y),
        static_cast<float>(ned_acceleration.z)};
      setpoint.yaw = static_cast<float>(yawEnuToNed(state.yaw));
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
    std::uint32_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand message;
    message.timestamp = timestampMicros();
    message.param1 = param1;
    message.param2 = param2;
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
  double hold_timeout_{0.3};
  double land_timeout_{1.0};
  Vec3 map_origin_;
  Mode mode_{Mode::DISABLED};
  navigation_message::Trajectory trajectory_;
  bool have_trajectory_{false};
  bool trajectory_started_{false};
  bool have_odometry_{false};
  bool armed_{false};
  bool offboard_{false};
  bool failsafe_{false};
  bool mode_command_sent_{false};
  bool arm_command_sent_{false};
  bool land_command_sent_{false};
  Vec3 current_position_;
  Vec3 hold_position_;
  Vec3 visual_velocity_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prestream_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mode_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_command_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_visual_velocity_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_intent_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<navigation_message::Trajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr visual_velocity_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
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
