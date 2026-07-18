// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <array>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace drone_navigation
{

class Px4StateAdapter : public rclcpp::Node
{
public:
  Px4StateAdapter()
  : Node("px4_state_adapter")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "drone_base_link");
    const auto origin = declare_parameter<std::vector<double>>(
      "px4_map_origin", {4.55, -0.38, 1.13});
    if (origin.size() != 3) {
      throw std::runtime_error("px4_map_origin must contain [x, y, z]");
    }
    origin_ = {origin[0], origin[1], origin[2]};

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20));
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/px4_status", rclcpp::QoS(10).transient_local());
    ack_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/px4_command_ack", rclcpp::QoS(10));
    landed_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/drone/navigation/landed", rclcpp::QoS(10).transient_local());

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    odometry_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      std::bind(&Px4StateAdapter::onOdometry, this, std::placeholders::_1));
    status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      std::bind(&Px4StateAdapter::onStatus, this, std::placeholders::_1));
    ack_subscription_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", px4_qos,
      std::bind(&Px4StateAdapter::onAck, this, std::placeholders::_1));
    land_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", px4_qos,
      std::bind(&Px4StateAdapter::onLandDetected, this, std::placeholders::_1));
  }

private:
  void onOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr message)
  {
    if (message->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED ||
      message->velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting PX4 odometry that is not NED position and NED velocity");
      return;
    }
    Px4OdometrySample input;
    for (std::size_t index = 0; index < 3; ++index) {
      input.position_ned[index] = message->position[index];
      input.velocity_ned[index] = message->velocity[index];
      input.angular_velocity_frd[index] = message->angular_velocity[index];
    }
    for (std::size_t index = 0; index < 4; ++index) {
      input.attitude_frd_to_ned_wxyz[index] = message->q[index];
    }
    auto output = px4NedFrdToRosEnuFlu(input);
    output.position_enu = output.position_enu + origin_;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = now();
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = output.position_enu.x;
    odometry.pose.pose.position.y = output.position_enu.y;
    odometry.pose.pose.position.z = output.position_enu.z;
    odometry.pose.pose.orientation.x = output.attitude_flu_to_enu.x;
    odometry.pose.pose.orientation.y = output.attitude_flu_to_enu.y;
    odometry.pose.pose.orientation.z = output.attitude_flu_to_enu.z;
    odometry.pose.pose.orientation.w = output.attitude_flu_to_enu.w;
    odometry.twist.twist.linear.x = output.velocity_enu.x;
    odometry.twist.twist.linear.y = output.velocity_enu.y;
    odometry.twist.twist.linear.z = output.velocity_enu.z;
    odometry.twist.twist.angular.x = output.angular_velocity_flu.x;
    odometry.twist.twist.angular.y = output.angular_velocity_flu.y;
    odometry.twist.twist.angular.z = output.angular_velocity_flu.z;
    for (std::size_t index = 0; index < 3; ++index) {
      odometry.pose.covariance[index * 7] = message->position_variance[index];
      odometry.twist.covariance[index * 7] = message->velocity_variance[index];
    }
    odometry_publisher_->publish(odometry);
  }

  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr message)
  {
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "ready=" << (message->pre_flight_checks_pass ? "true" : "false")
           << " armed=" <<
      (message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ? "true" : "false")
           << " offboard=" <<
      (message->nav_state ==
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD ? "true" : "false")
           << " failsafe=" << (message->failsafe ? "true" : "false")
           << " nav_state=" << static_cast<unsigned int>(message->nav_state);
    status.data = stream.str();
    status_publisher_->publish(status);
  }

  void onAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr message)
  {
    std_msgs::msg::String ack;
    ack.data = "command=" + std::to_string(message->command) +
      " result=" + std::to_string(message->result);
    ack_publisher_->publish(ack);
  }

  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr message)
  {
    std_msgs::msg::Bool landed;
    landed.data = message->landed;
    landed_publisher_->publish(landed);
  }

  std::string map_frame_;
  std::string base_frame_;
  Vec3 origin_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ack_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landed_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_subscription_;
};

}  // namespace drone_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_navigation::Px4StateAdapter>());
  rclcpp::shutdown();
  return 0;
}
