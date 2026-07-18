// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace drone_navigation
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const
  {
    const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(index.x) + 1048576);
    const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(index.y) + 1048576);
    const auto z = static_cast<std::uint64_t>(static_cast<std::int64_t>(index.z) + 1048576);
    return static_cast<std::size_t>((x * 73856093U) ^ (y * 19349663U) ^ (z * 83492791U));
  }
};

struct QuaternionWxyz
{
  double w;
  double x;
  double y;
  double z;
};

QuaternionWxyz multiply(const QuaternionWxyz & lhs, const QuaternionWxyz & rhs)
{
  return {
    lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
    lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
    lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
  };
}

QuaternionWxyz normalized(const QuaternionWxyz & value)
{
  const double magnitude = std::sqrt(
    value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
  if (magnitude < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  return {value.w / magnitude, value.x / magnitude, value.y / magnitude, value.z / magnitude};
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

GridIndex toGrid(const Vec3 & point, double resolution)
{
  return {
    static_cast<int>(std::llround(point.x / resolution)),
    static_cast<int>(std::llround(point.y / resolution)),
    static_cast<int>(std::llround(point.z / resolution)),
  };
}

Vec3 fromGrid(const GridIndex & index, double resolution)
{
  return {index.x * resolution, index.y * resolution, index.z * resolution};
}

double heuristic(const GridIndex & lhs, const GridIndex & rhs)
{
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  const double dz = static_cast<double>(lhs.z - rhs.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct QueueEntry
{
  double score;
  GridIndex index;

  bool operator<(const QueueEntry & other) const {return score > other.score;}
};

Vec3 weightedSum(
  const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3,
  const std::array<double, 4> & weights)
{
  return p0 * weights[0] + p1 * weights[1] + p2 * weights[2] + p3 * weights[3];
}

TrajectoryState sampleUniformBspline(
  const std::vector<Vec3> & controls, double duration, double seconds)
{
  if (controls.size() < 4 || duration <= 0.0) {
    return {};
  }
  const std::size_t segment_count = controls.size() - 3;
  const double bounded_time = clamp(seconds, 0.0, duration);
  const double curve_parameter = bounded_time / duration * static_cast<double>(segment_count);
  const std::size_t segment = std::min(
    static_cast<std::size_t>(curve_parameter), segment_count - 1);
  const double u = bounded_time >= duration ? 1.0 : curve_parameter - static_cast<double>(segment);
  const double u2 = u * u;
  const double u3 = u2 * u;

  const std::array<double, 4> position_weights{{
    (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0,
    (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0,
    (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0,
    u3 / 6.0,
  }};
  const std::array<double, 4> first_derivative_weights{{
    -0.5 * (1.0 - u) * (1.0 - u),
    1.5 * u2 - 2.0 * u,
    -1.5 * u2 + u + 0.5,
    0.5 * u2,
  }};
  const std::array<double, 4> second_derivative_weights{{
    1.0 - u,
    3.0 * u - 2.0,
    -3.0 * u + 1.0,
    u,
  }};

  const Vec3 & p0 = controls[segment];
  const Vec3 & p1 = controls[segment + 1];
  const Vec3 & p2 = controls[segment + 2];
  const Vec3 & p3 = controls[segment + 3];
  const double parameter_rate = static_cast<double>(segment_count) / duration;

  TrajectoryState state;
  state.position = weightedSum(p0, p1, p2, p3, position_weights);
  state.velocity = weightedSum(p0, p1, p2, p3, first_derivative_weights) * parameter_rate;
  state.acceleration = weightedSum(p0, p1, p2, p3, second_derivative_weights) *
    (parameter_rate * parameter_rate);
  if (norm(state.velocity) > 1e-6) {
    state.yaw = std::atan2(state.velocity.y, state.velocity.x);
  }
  return state;
}

}  // namespace

struct VoxelPlanner::Impl
{
  std::unordered_map<GridIndex, std::vector<Vec3>, GridIndexHash> obstacle_buckets;
  mutable std::unordered_map<GridIndex, bool, GridIndexHash> grid_occupancy_cache;
  double resolution;
  double collision_radius;
  int query_cell_radius;

  explicit Impl(const PlannerConfig & config)
  : resolution(config.resolution),
    collision_radius(std::max(config.inflation_radius, config.resolution * 0.5)),
    query_cell_radius(static_cast<int>(std::ceil(collision_radius / resolution)))
  {
  }

  bool occupied(const Vec3 & point) const
  {
    const GridIndex center = toGrid(point, resolution);
    for (int dx = -query_cell_radius; dx <= query_cell_radius; ++dx) {
      for (int dy = -query_cell_radius; dy <= query_cell_radius; ++dy) {
        for (int dz = -query_cell_radius; dz <= query_cell_radius; ++dz) {
          const GridIndex candidate{center.x + dx, center.y + dy, center.z + dz};
          const auto bucket = obstacle_buckets.find(candidate);
          if (bucket == obstacle_buckets.end()) {
            continue;
          }
          for (const auto & obstacle : bucket->second) {
            if (distance(point, obstacle) <= collision_radius) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  bool occupiedGrid(const GridIndex & index) const
  {
    const auto cached = grid_occupancy_cache.find(index);
    if (cached != grid_occupancy_cache.end()) {
      return cached->second;
    }
    const bool result = occupied(fromGrid(index, resolution));
    grid_occupancy_cache.emplace(index, result);
    return result;
  }
};

struct RollingVoxelMap::Impl
{
  struct Observation
  {
    Vec3 point;
    double stamp_seconds;
  };

  double resolution;
  double retention_seconds;
  std::unordered_map<GridIndex, Observation, GridIndexHash> observations;
};

Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3 & value, double scalar)
{
  return {value.x * scalar, value.y * scalar, value.z * scalar};
}

Vec3 operator/(const Vec3 & value, double scalar)
{
  if (std::abs(scalar) < 1e-12) {
    throw std::invalid_argument("cannot divide a vector by zero");
  }
  return value * (1.0 / scalar);
}

double norm(const Vec3 & value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double distance(const Vec3 & lhs, const Vec3 & rhs)
{
  return norm(lhs - rhs);
}

RosOdometrySample px4NedFrdToRosEnuFlu(const Px4OdometrySample & sample)
{
  RosOdometrySample converted;
  converted.position_enu = {
    sample.position_ned[1], sample.position_ned[0], -sample.position_ned[2]};
  converted.velocity_enu = {
    sample.velocity_ned[1], sample.velocity_ned[0], -sample.velocity_ned[2]};
  converted.angular_velocity_flu = {
    sample.angular_velocity_frd[0],
    -sample.angular_velocity_frd[1],
    -sample.angular_velocity_frd[2],
  };

  const double inverse_sqrt_two = 1.0 / std::sqrt(2.0);
  const QuaternionWxyz ned_to_enu{0.0, inverse_sqrt_two, inverse_sqrt_two, 0.0};
  const QuaternionWxyz frd_to_ned{
    sample.attitude_frd_to_ned_wxyz[0],
    sample.attitude_frd_to_ned_wxyz[1],
    sample.attitude_frd_to_ned_wxyz[2],
    sample.attitude_frd_to_ned_wxyz[3],
  };
  const QuaternionWxyz flu_to_frd{0.0, 1.0, 0.0, 0.0};
  const auto result = normalized(multiply(multiply(ned_to_enu, frd_to_ned), flu_to_frd));
  converted.attitude_flu_to_enu = {result.x, result.y, result.z, result.w};
  return converted;
}

Vec3 enuToNed(const Vec3 & value)
{
  return {value.y, value.x, -value.z};
}

double yawEnuToNed(double yaw_enu)
{
  double yaw_ned = kPi / 2.0 - yaw_enu;
  while (yaw_ned > kPi) {
    yaw_ned -= 2.0 * kPi;
  }
  while (yaw_ned < -kPi) {
    yaw_ned += 2.0 * kPi;
  }
  return yaw_ned;
}

RollingVoxelMap::RollingVoxelMap(double resolution, double retention_seconds)
{
  if (resolution <= 0.0 || retention_seconds <= 0.0) {
    throw std::invalid_argument("rolling voxel map resolution and retention must be positive");
  }
  impl_ = std::make_shared<Impl>();
  impl_->resolution = resolution;
  impl_->retention_seconds = retention_seconds;
}

void RollingVoxelMap::update(const std::vector<Vec3> & points, double stamp_seconds)
{
  for (const auto & point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      impl_->observations[toGrid(point, impl_->resolution)] = {point, stamp_seconds};
    }
  }
}

std::vector<Vec3> RollingVoxelMap::obstaclesAround(
  const Vec3 & center,
  double horizontal_range,
  double vertical_range,
  double stamp_seconds)
{
  std::vector<Vec3> result;
  for (auto iterator = impl_->observations.begin(); iterator != impl_->observations.end(); ) {
    if (stamp_seconds - iterator->second.stamp_seconds > impl_->retention_seconds) {
      iterator = impl_->observations.erase(iterator);
      continue;
    }
    const auto & point = iterator->second.point;
    if (std::hypot(point.x - center.x, point.y - center.y) <= horizontal_range &&
      std::abs(point.z - center.z) <= vertical_range * 0.5)
    {
      result.push_back(point);
    }
    ++iterator;
  }
  return result;
}

ExecutorSafetyAction executorSafetyAction(
  bool trajectory_expected,
  double odometry_age_seconds,
  double control_intent_age_seconds,
  double trajectory_age_seconds,
  double hold_timeout_seconds,
  double land_timeout_seconds)
{
  if (hold_timeout_seconds < 0.0 || land_timeout_seconds < hold_timeout_seconds) {
    throw std::invalid_argument("executor watchdog timeouts must be non-negative and ordered");
  }
  double worst_age = std::max(odometry_age_seconds, control_intent_age_seconds);
  if (trajectory_expected) {
    worst_age = std::max(worst_age, trajectory_age_seconds);
  }
  if (worst_age > land_timeout_seconds) {
    return ExecutorSafetyAction::LAND;
  }
  if (worst_age > hold_timeout_seconds) {
    return ExecutorSafetyAction::HOLD;
  }
  return ExecutorSafetyAction::CONTINUE;
}

VoxelPlanner::VoxelPlanner(PlannerConfig config)
: config_(config)
{
  if (config_.resolution <= 0.0 || config_.inflation_radius < 0.0) {
    throw std::invalid_argument("planner resolution must be positive and inflation non-negative");
  }
  impl_ = std::make_shared<Impl>(config_);
}

void VoxelPlanner::setObstacles(const std::vector<Vec3> & points)
{
  impl_->obstacle_buckets.clear();
  impl_->obstacle_buckets.reserve(points.size());
  impl_->grid_occupancy_cache.clear();
  for (const auto & point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      impl_->obstacle_buckets[toGrid(point, config_.resolution)].push_back(point);
    }
  }
}

bool VoxelPlanner::collisionFree(const Vec3 & start, const Vec3 & goal) const
{
  if (start.z < 0.0 || goal.z < 0.0 ||
    start.z > config_.virtual_ceiling || goal.z > config_.virtual_ceiling)
  {
    return false;
  }
  const double length = distance(start, goal);
  const int samples = std::max(1, static_cast<int>(std::ceil(length / (config_.resolution * 0.5))));
  for (int index = 0; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(samples);
    const Vec3 point = start + (goal - start) * ratio;
    if (impl_->occupied(point)) {
      return false;
    }
  }
  return true;
}

std::vector<Vec3> VoxelPlanner::plan(const Vec3 & start, const Vec3 & goal) const
{
  if (!std::isfinite(start.x) || !std::isfinite(goal.x) ||
    start.z < 0.0 || goal.z < 0.0 ||
    start.z > config_.virtual_ceiling || goal.z > config_.virtual_ceiling)
  {
    return {};
  }
  if (collisionFree(start, goal)) {
    return {start, goal};
  }

  const GridIndex start_index = toGrid(start, config_.resolution);
  const GridIndex goal_index = toGrid(goal, config_.resolution);
  const double margin_xy = config_.horizontal_range;
  const double margin_z = config_.vertical_range * 0.5;
  const double min_x = std::min(start.x, goal.x) - margin_xy;
  const double max_x = std::max(start.x, goal.x) + margin_xy;
  const double min_y = std::min(start.y, goal.y) - margin_xy;
  const double max_y = std::max(start.y, goal.y) + margin_xy;
  const double min_z = std::max(0.0, std::min(start.z, goal.z) - margin_z);
  const double max_z = std::min(
    config_.virtual_ceiling, std::max(start.z, goal.z) + margin_z);

  auto in_bounds = [&](const GridIndex & index) {
      const Vec3 point = fromGrid(index, config_.resolution);
      return point.x >= min_x && point.x <= max_x &&
             point.y >= min_y && point.y <= max_y &&
             point.z >= min_z && point.z <= max_z;
    };
  auto occupied = [&](const GridIndex & index) {
      return impl_->occupiedGrid(index);
    };

  std::priority_queue<QueueEntry> open;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed;
  g_score[start_index] = 0.0;
  open.push({heuristic(start_index, goal_index), start_index});

  constexpr std::size_t kMaximumExpandedVoxels = 500000;
  std::size_t expanded = 0;
  bool found = false;
  while (!open.empty() && expanded < kMaximumExpandedVoxels) {
    const GridIndex current = open.top().index;
    open.pop();
    if (closed.find(current) != closed.end()) {
      continue;
    }
    closed.insert(current);
    ++expanded;
    if (current == goal_index) {
      found = true;
      break;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
          if (!in_bounds(neighbor) || occupied(neighbor) || closed.find(neighbor) != closed.end()) {
            continue;
          }
          const double step_cost = std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz));
          const double tentative = g_score[current] + step_cost;
          const auto known = g_score.find(neighbor);
          if (known == g_score.end() || tentative < known->second) {
            g_score[neighbor] = tentative;
            came_from[neighbor] = current;
            open.push({tentative + heuristic(neighbor, goal_index), neighbor});
          }
        }
      }
    }
  }
  if (!found) {
    return {};
  }

  std::vector<Vec3> grid_path;
  GridIndex current = goal_index;
  grid_path.push_back(goal);
  while (!(current == start_index)) {
    const auto parent = came_from.find(current);
    if (parent == came_from.end()) {
      return {};
    }
    current = parent->second;
    if (!(current == start_index)) {
      grid_path.push_back(fromGrid(current, config_.resolution));
    }
  }
  grid_path.push_back(start);
  std::reverse(grid_path.begin(), grid_path.end());

  std::vector<Vec3> pruned;
  pruned.push_back(grid_path.front());
  std::size_t anchor = 0;
  while (anchor + 1 < grid_path.size()) {
    std::size_t furthest = anchor + 1;
    for (std::size_t candidate = grid_path.size() - 1; candidate > anchor + 1; --candidate) {
      if (collisionFree(grid_path[anchor], grid_path[candidate])) {
        furthest = candidate;
        break;
      }
    }
    pruned.push_back(grid_path[furthest]);
    anchor = furthest;
  }
  return pruned;
}

UniformBsplineTrajectory UniformBsplineTrajectory::fromWaypoints(
  const std::vector<Vec3> & waypoints, double max_velocity, double max_acceleration)
{
  if (waypoints.size() < 2 || max_velocity <= 0.0 || max_acceleration <= 0.0) {
    throw std::invalid_argument("trajectory needs two points and positive dynamic limits");
  }

  UniformBsplineTrajectory trajectory;
  trajectory.waypoints_.reserve(waypoints.size() + 4);
  trajectory.waypoints_.push_back(waypoints.front());
  trajectory.waypoints_.push_back(waypoints.front());
  trajectory.waypoints_.push_back(waypoints.front());
  for (std::size_t index = 1; index + 1 < waypoints.size(); ++index) {
    trajectory.waypoints_.push_back(waypoints[index]);
  }
  trajectory.waypoints_.push_back(waypoints.back());
  trajectory.waypoints_.push_back(waypoints.back());
  trajectory.waypoints_.push_back(waypoints.back());

  const std::size_t segment_count = trajectory.waypoints_.size() - 3;
  double max_first_derivative = 0.0;
  double max_second_derivative = 0.0;
  constexpr int kSamplesPerSegment = 50;
  for (std::size_t segment = 0; segment < segment_count; ++segment) {
    for (int sample_index = 0; sample_index <= kSamplesPerSegment; ++sample_index) {
      const double u = static_cast<double>(sample_index) / kSamplesPerSegment;
      const double u2 = u * u;
      const std::array<double, 4> d1{{
        -0.5 * (1.0 - u) * (1.0 - u), 1.5 * u2 - 2.0 * u,
        -1.5 * u2 + u + 0.5, 0.5 * u2}};
      const std::array<double, 4> d2{{1.0 - u, 3.0 * u - 2.0, -3.0 * u + 1.0, u}};
      const Vec3 first = weightedSum(
        trajectory.waypoints_[segment], trajectory.waypoints_[segment + 1],
        trajectory.waypoints_[segment + 2], trajectory.waypoints_[segment + 3], d1);
      const Vec3 second = weightedSum(
        trajectory.waypoints_[segment], trajectory.waypoints_[segment + 1],
        trajectory.waypoints_[segment + 2], trajectory.waypoints_[segment + 3], d2);
      max_first_derivative = std::max(max_first_derivative, norm(first));
      max_second_derivative = std::max(max_second_derivative, norm(second));
    }
  }
  const double segments = static_cast<double>(segment_count);
  const double velocity_duration = max_first_derivative * segments / max_velocity;
  const double acceleration_duration = std::sqrt(
    max_second_derivative * segments * segments / max_acceleration);
  trajectory.duration_ = std::max({velocity_duration, acceleration_duration, 0.1});
  trajectory.segment_start_times_.reserve(segment_count + 1);
  for (std::size_t index = 0; index <= segment_count; ++index) {
    trajectory.segment_start_times_.push_back(
      trajectory.duration_ * static_cast<double>(index) / segments);
  }
  return trajectory;
}

TrajectoryState UniformBsplineTrajectory::sample(double seconds) const
{
  return sampleUniformBspline(waypoints_, duration_, seconds);
}

double UniformBsplineTrajectory::duration() const
{
  return duration_;
}

bool UniformBsplineTrajectory::empty() const
{
  return waypoints_.empty();
}

std::string toString(FlightPhase phase)
{
  switch (phase) {
    case FlightPhase::IDLE: return "IDLE";
    case FlightPhase::PREFLIGHT: return "PREFLIGHT";
    case FlightPhase::ARMING: return "ARMING";
    case FlightPhase::TAKEOFF: return "TAKEOFF";
    case FlightPhase::EGO_TRANSIT: return "EGO_TRANSIT";
    case FlightPhase::TARGET_SEARCH: return "TARGET_SEARCH";
    case FlightPhase::VISUAL_ALIGN: return "VISUAL_ALIGN";
    case FlightPhase::DROP_HOLD: return "DROP_HOLD";
    case FlightPhase::RETURN: return "RETURN";
    case FlightPhase::LAND: return "LAND";
    case FlightPhase::COMPLETE: return "COMPLETE";
    case FlightPhase::HOLD: return "HOLD";
  }
  return "UNKNOWN";
}

SupervisorDecision FlightSupervisor::update(const SupervisorInputs & inputs)
{
  SupervisorDecision decision;
  const bool airborne_phase =
    phase_ == FlightPhase::TAKEOFF || phase_ == FlightPhase::EGO_TRANSIT ||
    phase_ == FlightPhase::TARGET_SEARCH || phase_ == FlightPhase::VISUAL_ALIGN ||
    phase_ == FlightPhase::DROP_HOLD || phase_ == FlightPhase::RETURN;
  const double worst_data_age = std::max(
    inputs.odometry_age_seconds, inputs.pointcloud_age_seconds);

  if (inputs.px4_failsafe && (airborne_phase || phase_ == FlightPhase::HOLD)) {
    phase_ = FlightPhase::HOLD;
    decision.phase = phase_;
    decision.request_land = true;
    decision.reason = "PX4 failsafe active";
    return decision;
  }

  if ((airborne_phase || phase_ == FlightPhase::HOLD) &&
    worst_data_age > inputs.land_timeout_seconds)
  {
    phase_ = FlightPhase::HOLD;
    decision.phase = phase_;
    decision.request_land = true;
    decision.reason = "odometry or point cloud missing for more than 1 second";
    return decision;
  }
  if (airborne_phase && worst_data_age > inputs.hold_timeout_seconds) {
    phase_ = FlightPhase::HOLD;
    decision.phase = phase_;
    decision.hold_position = true;
    decision.reason = "odometry or point cloud stale";
    return decision;
  }

  switch (phase_) {
    case FlightPhase::IDLE:
      if (inputs.mission_requested && inputs.ground_task_complete) {
        phase_ = FlightPhase::PREFLIGHT;
      }
      break;
    case FlightPhase::PREFLIGHT:
      decision.command_close_side_door = !inputs.side_door_closed;
      if (inputs.side_door_closed && inputs.px4_ready) {
        phase_ = FlightPhase::ARMING;
      }
      break;
    case FlightPhase::ARMING:
      decision.request_arm_offboard = true;
      if (inputs.armed && inputs.offboard) {
        phase_ = FlightPhase::TAKEOFF;
      }
      break;
    case FlightPhase::TAKEOFF:
      if (inputs.at_takeoff_pose) {
        phase_ = FlightPhase::EGO_TRANSIT;
      }
      break;
    case FlightPhase::EGO_TRANSIT:
      if (inputs.at_search_pose) {
        phase_ = FlightPhase::TARGET_SEARCH;
      }
      break;
    case FlightPhase::TARGET_SEARCH:
      if (inputs.target_visible) {
        phase_ = FlightPhase::VISUAL_ALIGN;
      }
      break;
    case FlightPhase::VISUAL_ALIGN:
      if (inputs.target_aligned) {
        phase_ = FlightPhase::DROP_HOLD;
      }
      break;
    case FlightPhase::DROP_HOLD:
      decision.command_open_bottom_door = true;
      if (inputs.payload_released) {
        phase_ = FlightPhase::RETURN;
      }
      break;
    case FlightPhase::RETURN:
      if (inputs.at_home) {
        phase_ = FlightPhase::LAND;
      }
      break;
    case FlightPhase::LAND:
      decision.request_land = true;
      if (inputs.landed) {
        phase_ = FlightPhase::COMPLETE;
      }
      break;
    case FlightPhase::COMPLETE:
      break;
    case FlightPhase::HOLD:
      decision.hold_position = true;
      break;
  }
  decision.phase = phase_;
  return decision;
}

FlightPhase FlightSupervisor::phase() const
{
  return phase_;
}

}  // namespace drone_navigation
