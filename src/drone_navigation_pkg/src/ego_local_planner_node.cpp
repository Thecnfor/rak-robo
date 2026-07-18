// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"
#include "drone_navigation_pkg/msg/trajectory.hpp"
#include "drone_navigation_pkg/msg/trajectory_point.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace drone_navigation
{
namespace navigation_message = drone_navigation_pkg::msg;

class EgoLocalPlannerNode : public rclcpp::Node
{
public:
  EgoLocalPlannerNode()
  : Node("ego_local_planner"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    PlannerConfig config;
    config.resolution = declare_parameter<double>("voxel_resolution", 0.10);
    config.inflation_radius = declare_parameter<double>("obstacle_inflation", 0.25);
    config.horizontal_range = declare_parameter<double>("local_horizontal_range", 5.5);
    config.vertical_range = declare_parameter<double>("local_vertical_range", 4.5);
    config.virtual_ceiling = declare_parameter<double>("virtual_ceiling", 2.9);
    config.max_velocity = declare_parameter<double>("max_velocity", 0.5);
    config.max_acceleration = declare_parameter<double>("max_acceleration", 1.0);
    obstacle_memory_seconds_ = declare_parameter<double>("obstacle_memory_seconds", 1.0);
    config_ = config;
    planner_ = std::make_unique<VoxelPlanner>(config_);
    rolling_map_ = std::make_unique<RollingVoxelMap>(
      config_.resolution, obstacle_memory_seconds_);

    trajectory_publisher_ = create_publisher<navigation_message::Trajectory>(
      "/drone/navigation/trajectory", rclcpp::QoS(1).transient_local());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/navigation/planned_path", rclcpp::QoS(1).transient_local());
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/drone/navigation/planner_state", rclcpp::QoS(10).transient_local());

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/navigation/odometry", rclcpp::QoS(20),
      std::bind(&EgoLocalPlannerNode::onOdometry, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/navigation/goal", rclcpp::QoS(1).transient_local(),
      std::bind(&EgoLocalPlannerNode::onGoal, this, std::placeholders::_1));
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/avoidance/lidar/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&EgoLocalPlannerNode::onPointcloud, this, std::placeholders::_1));
    replan_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&EgoLocalPlannerNode::replan, this));
  }

private:
  void publishState(const std::string & text)
  {
    std_msgs::msg::String state;
    state.data = text;
    state_publisher_->publish(state);
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    current_position_ = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z,
    };
    last_odometry_time_ = now();
    have_odometry_ = true;
    odometry_revision_++;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      publishState("REJECTED goal_frame_must_be_" + map_frame_);
      return;
    }
    goal_ = {message->pose.position.x, message->pose.position.y, message->pose.position.z};
    have_goal_ = true;
    goal_revision_++;
  }

  void onPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    try {
      if (message->header.frame_id == map_frame_) {
        cloud_in_map = *message;
      } else {
        const auto transform = tf_buffer_.lookupTransform(
          map_frame_, message->header.frame_id, message->header.stamp,
          std::chrono::milliseconds(50));
        tf2::doTransform(*message, cloud_in_map, transform);
      }
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Point cloud transform unavailable: %s",
        exception.what());
      return;
    }

    std::vector<Vec3> obstacles;
    obstacles.reserve(cloud_in_map.width * cloud_in_map.height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud_in_map, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud_in_map, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud_in_map, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        if (have_odometry_ &&
          (std::hypot(*x - current_position_.x, *y - current_position_.y) >
          config_.horizontal_range ||
          std::abs(*z - current_position_.z) > config_.vertical_range * 0.5))
        {
          continue;
        }
        obstacles.push_back({*x, *y, *z});
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_WARN(get_logger(), "Invalid PointCloud2 fields: %s", exception.what());
      return;
    }
    last_pointcloud_time_ = now();
    rolling_map_->update(obstacles, last_pointcloud_time_.seconds());
    planner_->setObstacles(
      rolling_map_->obstaclesAround(
        current_position_, config_.horizontal_range, config_.vertical_range,
        last_pointcloud_time_.seconds()));
    have_pointcloud_ = true;
    cloud_revision_++;
  }

  void replan()
  {
    if (!have_odometry_ || !have_pointcloud_ || !have_goal_) {
      publishState("WAITING_FOR_INPUTS");
      return;
    }
    if ((now() - last_odometry_time_).seconds() > 0.3 ||
      (now() - last_pointcloud_time_).seconds() > 0.3)
    {
      publishState("HOLD_STALE_INPUT");
      return;
    }
    if (goal_revision_ == planned_goal_revision_ && cloud_revision_ == planned_cloud_revision_ &&
      odometry_revision_ == planned_odometry_revision_)
    {
      return;
    }

    const auto path = planner_->plan(current_position_, localGoal());
    planned_goal_revision_ = goal_revision_;
    planned_cloud_revision_ = cloud_revision_;
    planned_odometry_revision_ = odometry_revision_;
    if (path.size() < 2) {
      publishState("NO_PATH");
      return;
    }

    const auto spline = UniformBsplineTrajectory::fromWaypoints(
      path, config_.max_velocity, config_.max_acceleration);
    bool collision_free = true;
    Vec3 previous = spline.sample(0.0).position;
    for (double time = 0.05; time <= spline.duration(); time += 0.05) {
      const Vec3 position = spline.sample(time).position;
      if (!planner_->collisionFree(previous, position)) {
        collision_free = false;
        break;
      }
      previous = position;
    }
    if (!collision_free) {
      publishState("NO_PATH_SPLINE_COLLISION");
      return;
    }

    const auto stamp = now();
    nav_msgs::msg::Path debug_path;
    debug_path.header.stamp = stamp;
    debug_path.header.frame_id = map_frame_;
    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = debug_path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      debug_path.poses.push_back(pose);
    }
    path_publisher_->publish(debug_path);

    navigation_message::Trajectory trajectory;
    trajectory.header = debug_path.header;
    trajectory.trajectory_id = ++trajectory_id_;
    constexpr double kSamplePeriod = 0.05;
    for (double time = 0.0; time < spline.duration(); time += kSamplePeriod) {
      appendTrajectoryPoint(trajectory, spline.sample(time), time);
    }
    appendTrajectoryPoint(trajectory, spline.sample(spline.duration()), spline.duration());
    trajectory_publisher_->publish(trajectory);
    publishState("ACTIVE trajectory_id=" + std::to_string(trajectory_id_));
  }

  Vec3 localGoal() const
  {
    Vec3 delta = goal_ - current_position_;
    const double horizontal_distance = std::hypot(delta.x, delta.y);
    if (horizontal_distance > config_.horizontal_range) {
      const double scale = config_.horizontal_range / horizontal_distance;
      delta.x *= scale;
      delta.y *= scale;
    }
    const double half_vertical_range = config_.vertical_range * 0.5;
    delta.z = std::clamp(delta.z, -half_vertical_range, half_vertical_range);
    Vec3 local_goal = current_position_ + delta;
    local_goal.z = std::clamp(local_goal.z, 0.0, config_.virtual_ceiling);
    return local_goal;
  }

  static void appendTrajectoryPoint(
    navigation_message::Trajectory & trajectory, const TrajectoryState & state, double seconds)
  {
    navigation_message::TrajectoryPoint point;
    const auto nanoseconds = static_cast<std::int64_t>(seconds * 1e9);
    point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    point.position.x = state.position.x;
    point.position.y = state.position.y;
    point.position.z = state.position.z;
    point.velocity.x = state.velocity.x;
    point.velocity.y = state.velocity.y;
    point.velocity.z = state.velocity.z;
    point.acceleration.x = state.acceleration.x;
    point.acceleration.y = state.acceleration.y;
    point.acceleration.z = state.acceleration.z;
    point.yaw = static_cast<float>(state.yaw);
    trajectory.points.push_back(point);
  }

  std::string map_frame_;
  PlannerConfig config_;
  double obstacle_memory_seconds_{1.0};
  std::unique_ptr<VoxelPlanner> planner_;
  std::unique_ptr<RollingVoxelMap> rolling_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Vec3 current_position_;
  Vec3 goal_;
  bool have_odometry_{false};
  bool have_pointcloud_{false};
  bool have_goal_{false};
  std::uint64_t cloud_revision_{0};
  std::uint64_t goal_revision_{0};
  std::uint64_t odometry_revision_{0};
  std::uint64_t planned_cloud_revision_{0};
  std::uint64_t planned_goal_revision_{0};
  std::uint64_t planned_odometry_revision_{0};
  std::uint32_t trajectory_id_{0};
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pointcloud_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<navigation_message::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

}  // namespace drone_navigation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_navigation::EgoLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
