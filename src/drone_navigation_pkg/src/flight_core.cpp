// Copyright 2026 Competition Team
// SPDX-License-Identifier: GPL-3.0-only
#include "drone_navigation_pkg/flight_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace drone_navigation
{

std::uint64_t nextMonotonicTimestampMicros(
  std::uint64_t proposed_timestamp,
  std::uint64_t previous_timestamp)
{
  if (proposed_timestamp > previous_timestamp) {
    return proposed_timestamp;
  }
  if (previous_timestamp == std::numeric_limits<std::uint64_t>::max()) {
    return previous_timestamp;
  }
  return previous_timestamp + 1U;
}

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

PositionControlSetpoint fixedDiagnosticControlSetpoint(
  const TrajectoryState & state_ned,
  bool vertical_only)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!vertical_only) {
    return {
      {state_ned.position.x, state_ned.position.y, state_ned.position.z},
      {state_ned.velocity.x, state_ned.velocity.y, state_ned.velocity.z},
      {state_ned.acceleration.x, state_ned.acceleration.y, state_ned.acceleration.z},
      state_ned.yaw,
      nan,
    };
  }

  // The launch cradle supplies the horizontal constraint before the handoff
  // clearance. Do not let local-position or heading estimator
  // drift wind up the corresponding controllers while the vehicle is guided.
  // Finite zero XY acceleration keeps PX4's per-axis validity contract
  // satisfied without enabling the horizontal velocity PID integrator. A
  // zero yaw-rate command avoids accumulating heading error while the guide
  // mechanically prevents yaw; the executor captures the current heading at
  // release for a bumpless handoff to attitude control.
  return {
    {nan, nan, state_ned.position.z},
    {nan, nan, state_ned.velocity.z},
    {0.0, 0.0, state_ned.acceleration.z},
    nan,
    0.0,
  };
}

bool verticalOnlyDiagnosticActive(
  bool enabled,
  bool currently_active,
  bool inside_guided_region,
  double current_clearance_m,
  double release_clearance_m,
  double reengage_clearance_m)
{
  if (!enabled) {
    return false;
  }
  if (!std::isfinite(current_clearance_m) ||
    !std::isfinite(release_clearance_m) ||
    !std::isfinite(reengage_clearance_m) ||
    reengage_clearance_m < 0.0 ||
    release_clearance_m <= reengage_clearance_m)
  {
    return true;
  }
  if (currently_active) {
    // Once armed in the guide, horizontal drift is evidence of contact with a
    // guide wall, not evidence that the aircraft has cleared it. Keep
    // horizontal control disabled only until the configured vertical
    // clearance: that threshold must be below the physical guide top so PX4
    // has a contact margin in which to capture XY before free flight.
    return current_clearance_m < release_clearance_m;
  }
  return inside_guided_region && current_clearance_m <= reengage_clearance_m;
}

double trustedLiftClearance(
  double estimated_clearance_m,
  bool truth_valid,
  double truth_clearance_m)
{
  const bool estimated_valid = std::isfinite(estimated_clearance_m);
  const bool physical_valid = truth_valid && std::isfinite(truth_clearance_m);
  // In the Isaac diagnostic the fresh raw pose is the physical lift witness.
  // PX4 local-Z can drift by more than the 5 mm release threshold while the
  // airframe is still resting on the support, so taking the maximum can
  // release horizontal control before physical separation. Fall back to the
  // estimate only when the diagnostic witness is unavailable.
  if (physical_valid) {
    return truth_clearance_m;
  }
  if (estimated_valid) {
    return estimated_clearance_m;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

bool verticalOnlyHandoffConfigurationSafe(
  double release_clearance_m,
  double reengage_clearance_m,
  double physical_guide_height_m,
  double minimum_handoff_lead_m)
{
  const bool finite =
    std::isfinite(release_clearance_m) &&
    std::isfinite(reengage_clearance_m) &&
    std::isfinite(physical_guide_height_m) &&
    std::isfinite(minimum_handoff_lead_m);
  return finite &&
         reengage_clearance_m >= 0.0 &&
         release_clearance_m > reengage_clearance_m &&
         physical_guide_height_m > 0.0 &&
         minimum_handoff_lead_m >= 0.0 &&
         release_clearance_m + minimum_handoff_lead_m <=
         physical_guide_height_m + 1e-9;
}

double fixedHandoffBlendScale(double elapsed_seconds, double blend_seconds)
{
  if (!std::isfinite(elapsed_seconds) || !std::isfinite(blend_seconds) ||
    blend_seconds <= 0.0)
  {
    return 0.0;
  }
  return std::clamp(1.0 - std::max(0.0, elapsed_seconds) / blend_seconds, 0.0, 1.0);
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

bool prearmPoseAllowed(
  const PrearmPoseSample & sample,
  const PrearmPoseLimits & limits)
{
  if (limits.position_tolerance < 0.0 || limits.max_speed < 0.0 ||
    limits.max_tilt_radians < 0.0)
  {
    return false;
  }
  return distance(sample.position, limits.expected_position) <= limits.position_tolerance &&
         norm(sample.velocity) <= limits.max_speed &&
         std::abs(sample.roll_radians) <= limits.max_tilt_radians &&
         std::abs(sample.pitch_radians) <= limits.max_tilt_radians;
}

bool freshPlannerMapReady(
  bool state_received,
  bool map_ready,
  double state_age_seconds,
  double timeout_seconds)
{
  return timeout_seconds >= 0.0 && state_age_seconds >= 0.0 &&
         state_received && map_ready && state_age_seconds <= timeout_seconds;
}

bool fixedSetpointReady(
  bool diagnostic_enabled,
  bool setpoint_received,
  double setpoint_age_seconds,
  double timeout_seconds)
{
  return diagnostic_enabled && setpoint_received && timeout_seconds >= 0.0 &&
         setpoint_age_seconds >= 0.0 && setpoint_age_seconds <= timeout_seconds;
}

std::optional<bool> boolTokenValue(
  const std::string & text,
  const std::string & key)
{
  const std::string prefix = key + "=";
  std::optional<bool> result;
  std::istringstream stream(text);
  std::string token;
  while (stream >> token) {
    if (token.rfind(prefix, 0) != 0) {
      continue;
    }
    std::optional<bool> value;
    if (token == prefix + "true") {
      value = true;
    } else if (token == prefix + "false") {
      value = false;
    } else {
      return std::nullopt;
    }
    if (result.has_value() && result != value) {
      return std::nullopt;
    }
    result = value;
  }
  return result;
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

Vec3 visualAlignmentVelocityEnu(
  double normalized_image_x,
  double normalized_image_y,
  double vehicle_yaw_enu,
  double proportional_gain,
  double maximum_speed)
{
  if (!std::isfinite(normalized_image_x) || !std::isfinite(normalized_image_y) ||
    !std::isfinite(vehicle_yaw_enu) || !std::isfinite(proportional_gain) ||
    !std::isfinite(maximum_speed) || proportional_gain < 0.0 || maximum_speed <= 0.0)
  {
    throw std::invalid_argument("visual alignment inputs and limits must be finite");
  }

  // The Pegasus camera looks along body -Z. Image +X is body +X, while image
  // +Y points toward body -Y. Rotate that body-frame correction into map ENU.
  const double body_x = proportional_gain * normalized_image_x;
  const double body_y = -proportional_gain * normalized_image_y;
  const double cosine = std::cos(vehicle_yaw_enu);
  const double sine = std::sin(vehicle_yaw_enu);
  return {
    std::clamp(cosine * body_x - sine * body_y, -maximum_speed, maximum_speed),
    std::clamp(sine * body_x + cosine * body_y, -maximum_speed, maximum_speed),
    0.0,
  };
}

bool visualTargetRecent(
  bool have_valid_detection,
  double valid_detection_age_seconds,
  double loss_grace_seconds)
{
  return have_valid_detection &&
         std::isfinite(valid_detection_age_seconds) &&
         std::isfinite(loss_grace_seconds) &&
         valid_detection_age_seconds >= 0.0 &&
         loss_grace_seconds >= 0.0 &&
         valid_detection_age_seconds <= loss_grace_seconds;
}

Vec3 positionAlignmentVelocityEnu(
  const Vec3 & current_position,
  const Vec3 & target_position,
  double proportional_gain,
  double maximum_horizontal_speed,
  double maximum_vertical_speed)
{
  const std::array<double, 9> values{{
      current_position.x, current_position.y, current_position.z,
      target_position.x, target_position.y, target_position.z,
      proportional_gain, maximum_horizontal_speed, maximum_vertical_speed,
    }};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || proportional_gain < 0.0 || maximum_horizontal_speed <= 0.0 ||
    maximum_vertical_speed <= 0.0)
  {
    throw std::invalid_argument("position alignment inputs and limits must be valid");
  }
  Vec3 velocity = (target_position - current_position) * proportional_gain;
  const double horizontal_speed = std::hypot(velocity.x, velocity.y);
  if (horizontal_speed > maximum_horizontal_speed) {
    const double scale = maximum_horizontal_speed / horizontal_speed;
    velocity.x *= scale;
    velocity.y *= scale;
  }
  velocity.z = std::clamp(
    velocity.z, -maximum_vertical_speed, maximum_vertical_speed);
  return velocity;
}

Vec3 rawErrorCorrectedNavigationTarget(
  const Vec3 & raw_target,
  const Vec3 & raw_position,
  const Vec3 & navigation_position,
  double horizontal_gain)
{
  const std::array<double, 10> values{{
      raw_target.x, raw_target.y, raw_target.z,
      raw_position.x, raw_position.y, raw_position.z,
      navigation_position.x, navigation_position.y, navigation_position.z,
      horizontal_gain,
    }};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || horizontal_gain <= 0.0 || horizontal_gain > 3.0)
  {
    throw std::invalid_argument("return guidance requires finite vectors and gain in (0, 3]");
  }
  return {
    navigation_position.x + horizontal_gain * (raw_target.x - raw_position.x),
    navigation_position.y + horizontal_gain * (raw_target.y - raw_position.y),
    navigation_position.z + raw_target.z - raw_position.z,
  };
}

Vec3 stagedReturnRawTarget(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  double transit_clearance,
  double approach_clearance,
  double descent_radius)
{
  const std::array<double, 9> values{{
      raw_home.x, raw_home.y, raw_home.z,
      raw_position.x, raw_position.y, raw_position.z,
      transit_clearance, approach_clearance, descent_radius,
    }};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || transit_clearance <= approach_clearance || approach_clearance <= 0.0 ||
    descent_radius <= 0.0)
  {
    throw std::invalid_argument(
            "staged return requires transit clearance above approach clearance "
            "and a positive descent radius");
  }
  const double horizontal_error = std::hypot(
    raw_position.x - raw_home.x, raw_position.y - raw_home.y);
  const double clearance =
    horizontal_error <= descent_radius ? approach_clearance : transit_clearance;
  return {raw_home.x, raw_home.y, raw_home.z + clearance};
}

bool returnTransitWaypointReached(
  const Vec3 & raw_position,
  const Vec3 & transit_waypoint,
  double horizontal_tolerance,
  double vertical_tolerance)
{
  const std::array<double, 8> values{{
      raw_position.x, raw_position.y, raw_position.z,
      transit_waypoint.x, transit_waypoint.y, transit_waypoint.z,
      horizontal_tolerance, vertical_tolerance,
    }};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || horizontal_tolerance <= 0.0 || vertical_tolerance <= 0.0)
  {
    throw std::invalid_argument(
            "return transit waypoint requires finite vectors and positive tolerances");
  }
  return std::hypot(
    raw_position.x - transit_waypoint.x,
    raw_position.y - transit_waypoint.y) <= horizontal_tolerance &&
         std::abs(raw_position.z - transit_waypoint.z) <= vertical_tolerance;
}

bool shouldResetReturnTransitWaypoint(
  FlightPhase previous_phase,
  FlightPhase current_phase)
{
  return previous_phase == FlightPhase::DROP_HOLD &&
         current_phase == FlightPhase::RETURN;
}

bool returnFineAlignmentReady(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  double horizontal_radius)
{
  const std::array<double, 7> values{{
      raw_home.x, raw_home.y, raw_home.z,
      raw_position.x, raw_position.y, raw_position.z,
      horizontal_radius,
    }};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || horizontal_radius <= 0.0)
  {
    throw std::invalid_argument(
            "return fine alignment requires finite vectors and a positive radius");
  }
  return std::hypot(
    raw_position.x - raw_home.x,
    raw_position.y - raw_home.y) <= horizontal_radius;
}

Vec3 returnRouteRawTarget(
  const Vec3 & raw_home,
  const Vec3 & raw_position,
  const Vec3 & transit_waypoint,
  const Vec3 & approach_waypoint,
  bool transit_waypoint_reached,
  double fine_alignment_radius,
  double transit_clearance,
  double approach_clearance,
  double descent_radius)
{
  if (!transit_waypoint_reached) {
    const std::array<double, 3> waypoint_values{{
        transit_waypoint.x, transit_waypoint.y, transit_waypoint.z,
      }};
    if (!std::all_of(waypoint_values.begin(), waypoint_values.end(), [](double value) {
        return std::isfinite(value);
      }))
    {
      throw std::invalid_argument("return transit waypoint must be finite");
    }
    return transit_waypoint;
  }
  const std::array<double, 3> approach_values{{
      approach_waypoint.x, approach_waypoint.y, approach_waypoint.z,
    }};
  if (!std::all_of(approach_values.begin(), approach_values.end(), [](double value) {
      return std::isfinite(value);
    }) || !returnFineAlignmentReady(
      raw_home, approach_waypoint, fine_alignment_radius))
  {
    throw std::invalid_argument(
            "return approach waypoint must be finite and inside the fine alignment radius");
  }
  if (!returnFineAlignmentReady(raw_home, raw_position, fine_alignment_radius)) {
    return approach_waypoint;
  }
  return stagedReturnRawTarget(
    raw_home, raw_position, transit_clearance, approach_clearance, descent_radius);
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

double trajectoryControlSourceAge(
  bool have_trajectory,
  bool trajectory_started,
  double trajectory_receipt_age_seconds)
{
  if (!have_trajectory) {
    return std::numeric_limits<double>::infinity();
  }
  // A trajectory is a complete time-parametrized control intent. Once
  // execution starts, sampling beyond its duration intentionally clamps to
  // the final point, which is a safe position hold. Slow replanning must not
  // invalidate that already accepted intent.
  if (trajectory_started) {
    return 0.0;
  }
  return std::max(0.0, trajectory_receipt_age_seconds);
}

double trajectoryReplacementDelay(
  double configured_minimum_seconds,
  double active_trajectory_duration_seconds)
{
  if (!std::isfinite(configured_minimum_seconds) ||
    !std::isfinite(active_trajectory_duration_seconds) ||
    configured_minimum_seconds < 0.0 || active_trajectory_duration_seconds < 0.0)
  {
    throw std::invalid_argument("trajectory replacement delays must be finite and non-negative");
  }
  // Normal same-goal updates stay pinned through the endpoint so a high-rate
  // rolling planner cannot repeatedly restart the zero-speed beginning of a
  // trajectory. Phase changes, NO_PATH recovery, and return-altitude safety
  // use explicit preemption paths in the executor.
  return std::max(configured_minimum_seconds, active_trajectory_duration_seconds);
}

bool trajectoryMinimumAltitudeImproves(
  double current_minimum_altitude,
  double incoming_minimum_altitude,
  double required_improvement)
{
  if (!std::isfinite(current_minimum_altitude) ||
    !std::isfinite(incoming_minimum_altitude) ||
    !std::isfinite(required_improvement) || required_improvement < 0.0)
  {
    throw std::invalid_argument(
            "trajectory altitude comparison requires finite values and a "
            "non-negative improvement");
  }
  return incoming_minimum_altitude >=
         current_minimum_altitude + required_improvement;
}

bool plannerRecoveryAllowsTrajectoryReplacement(
  const std::string & previous_state,
  const std::string & current_state)
{
  const bool was_blocked = previous_state.rfind("NO_PATH", 0) == 0;
  const bool now_active = current_state.rfind("ACTIVE", 0) == 0;
  return was_blocked && now_active;
}

bool forcedTrajectoryEndpointChanged(
  bool have_current_trajectory,
  const Vec3 & current_endpoint,
  const Vec3 & incoming_endpoint,
  double endpoint_tolerance)
{
  if (!std::isfinite(endpoint_tolerance) || endpoint_tolerance < 0.0) {
    throw std::invalid_argument("trajectory endpoint tolerance must be finite and non-negative");
  }
  return !have_current_trajectory ||
         distance(current_endpoint, incoming_endpoint) > endpoint_tolerance;
}

bool operatorArmRequestAllowed(
  bool have_goal,
  bool side_door_closed,
  bool px4_inputs_ready,
  bool prearm_pose_allowed,
  bool landed_known,
  bool landed,
  bool armed,
  bool offboard)
{
  if (!have_goal || !side_door_closed || !px4_inputs_ready || !landed_known) {
    return false;
  }
  const bool safe_ground_arm = landed && !armed && prearm_pose_allowed;
  // Land detection can remain true for several cycles after successful arming.
  // Once PX4 reports armed+offboard, never interrupt the stream because of that lag.
  const bool active_offboard_tracking = armed && offboard;
  return safe_ground_arm || active_offboard_tracking;
}

bool prearmAttitudeAgreementAllowed(
  double raw_roll_radians,
  double raw_pitch_radians,
  double estimated_roll_radians,
  double estimated_pitch_radians,
  double maximum_error_radians)
{
  if (!std::isfinite(raw_roll_radians) || !std::isfinite(raw_pitch_radians) ||
    !std::isfinite(estimated_roll_radians) || !std::isfinite(estimated_pitch_radians) ||
    !std::isfinite(maximum_error_radians) || maximum_error_radians < 0.0)
  {
    return false;
  }
  const double roll_error = std::abs(
    std::remainder(estimated_roll_radians - raw_roll_radians, 2.0 * kPi));
  const double pitch_error = std::abs(estimated_pitch_radians - raw_pitch_radians);
  return roll_error <= maximum_error_radians && pitch_error <= maximum_error_radians;
}

bool px4DiscreteStateUsable(
  bool state_received,
  double continuous_transport_age_seconds,
  double transport_timeout_seconds)
{
  return state_received && continuous_transport_age_seconds <= transport_timeout_seconds;
}

bool updateSideDoorClosed(bool current_state, const std::string & cargo_status)
{
  if (cargo_status.find("left_closed") != std::string::npos ||
    cargo_status.find("side_closed") != std::string::npos)
  {
    return true;
  }
  if (cargo_status.find("left_opened") != std::string::npos ||
    cargo_status.find("side_opened") != std::string::npos)
  {
    return false;
  }
  return current_state;
}

bool shouldAcceptTrajectoryUpdate(
  bool trajectory_started,
  bool armed,
  bool offboard,
  double accepted_trajectory_age_seconds,
  double minimum_execution_seconds)
{
  return !trajectory_started || !armed || !offboard ||
         accepted_trajectory_age_seconds >= minimum_execution_seconds;
}

bool shouldRequestGroundDisarm(
  ExecutorFlightState state,
  bool armed,
  bool auto_land,
  bool landed,
  bool landed_after_latch,
  double landed_duration_seconds,
  double landing_state_duration_seconds,
  double minimum_ground_delay_seconds)
{
  if (minimum_ground_delay_seconds < 0.0) {
    throw std::invalid_argument("ground disarm delay must be non-negative");
  }
  return state == ExecutorFlightState::LAND_LATCHED && armed && auto_land && landed &&
         landed_after_latch &&
         landed_duration_seconds >= minimum_ground_delay_seconds &&
         landing_state_duration_seconds >= minimum_ground_delay_seconds;
}

bool forceDisarmDiagnosticAllowed(
  bool diagnostic_enabled,
  ExecutorFlightState state,
  bool armed,
  bool auto_land)
{
  return diagnostic_enabled && state == ExecutorFlightState::LAND_LATCHED &&
         armed && auto_land;
}

bool forceDisarmBypassesLandLatch(
  bool diagnostic_enabled,
  bool land_latched,
  const std::string & operator_mode)
{
  return diagnostic_enabled && land_latched && operator_mode == "FORCE_DISARM";
}

bool armCommandAllowed(
  bool arm_requested,
  bool preflight_accepted,
  bool failsafe,
  double previous_command_age_seconds)
{
  return arm_requested && preflight_accepted && !failsafe &&
         previous_command_age_seconds >= 1.0;
}

ExecutorRequestedMode reduceExecutorRequest(
  ExecutorRequestedMode current,
  ExecutorRequestedMode incoming,
  ExecutorFlightState state)
{
  if (state == ExecutorFlightState::DISABLED && incoming == ExecutorRequestedMode::LAND) {
    return ExecutorRequestedMode::DISABLED;
  }
  if (state == ExecutorFlightState::DISABLED && incoming == ExecutorRequestedMode::RESET) {
    return incoming;
  }
  if (state == ExecutorFlightState::COMPLETE && incoming == ExecutorRequestedMode::RESET) {
    return incoming;
  }
  if (current == ExecutorRequestedMode::LAND ||
    state == ExecutorFlightState::LAND_LATCHED)
  {
    return ExecutorRequestedMode::LAND;
  }
  return incoming;
}

ExecutorLifecycleDecision ExecutorLifecycle::update(const ExecutorLifecycleInputs & inputs)
{
  const bool was_complete = state_ == ExecutorFlightState::COMPLETE;
  if ((inputs.requested_mode == ExecutorRequestedMode::LAND || inputs.failsafe) &&
    state_ != ExecutorFlightState::DISABLED && state_ != ExecutorFlightState::COMPLETE)
  {
    state_ = ExecutorFlightState::LAND_LATCHED;
  }

  switch (state_) {
    case ExecutorFlightState::DISABLED:
      if (inputs.requested_mode == ExecutorRequestedMode::ARM_TRAJECTORY) {
        state_ = ExecutorFlightState::PRESTREAM;
      }
      break;
    case ExecutorFlightState::PRESTREAM:
      if (inputs.requested_mode == ExecutorRequestedMode::DISABLED && !inputs.armed) {
        state_ = ExecutorFlightState::DISABLED;
        break;
      }
      if (inputs.armed && inputs.offboard) {
        state_ = ExecutorFlightState::ACTIVE;
      }
      break;
    case ExecutorFlightState::ACTIVE:
      if (inputs.requested_mode == ExecutorRequestedMode::HOLD) {
        state_ = ExecutorFlightState::HOLD;
      } else if (inputs.requested_mode == ExecutorRequestedMode::DISABLED) {
        state_ = inputs.armed ? ExecutorFlightState::LAND_LATCHED :
          ExecutorFlightState::DISABLED;
      }
      break;
    case ExecutorFlightState::HOLD:
      if ((inputs.requested_mode == ExecutorRequestedMode::TRAJECTORY ||
        inputs.requested_mode == ExecutorRequestedMode::VISUAL) && inputs.armed)
      {
        state_ = ExecutorFlightState::ACTIVE;
      } else if (inputs.requested_mode == ExecutorRequestedMode::DISABLED) {
        state_ = inputs.armed ? ExecutorFlightState::LAND_LATCHED :
          ExecutorFlightState::DISABLED;
      }
      break;
    case ExecutorFlightState::LAND_LATCHED:
      if (inputs.landed_known && inputs.landed && !inputs.armed) {
        state_ = ExecutorFlightState::COMPLETE;
      }
      break;
    case ExecutorFlightState::COMPLETE:
      if (was_complete && inputs.requested_mode == ExecutorRequestedMode::RESET &&
        inputs.landed_known && inputs.landed && !inputs.armed)
      {
        state_ = ExecutorFlightState::DISABLED;
      }
      break;
  }

  ExecutorLifecycleDecision decision;
  decision.state = state_;
  switch (state_) {
    case ExecutorFlightState::PRESTREAM:
      decision.stream_offboard = true;
      decision.request_offboard = inputs.prestream_complete && !inputs.offboard;
      decision.request_arm = inputs.prestream_complete && inputs.offboard && !inputs.armed;
      break;
    case ExecutorFlightState::ACTIVE:
    case ExecutorFlightState::HOLD:
      decision.stream_offboard = inputs.armed && !inputs.failsafe;
      break;
    case ExecutorFlightState::LAND_LATCHED:
      decision.stream_offboard = inputs.armed && !inputs.auto_land && !inputs.failsafe;
      decision.request_land = inputs.armed && !inputs.auto_land;
      break;
    case ExecutorFlightState::DISABLED:
    case ExecutorFlightState::COMPLETE:
      break;
  }
  decision.request_loiter =
    inputs.requested_mode == ExecutorRequestedMode::RESET &&
    state_ == ExecutorFlightState::DISABLED && inputs.landed_known &&
    inputs.landed && !inputs.armed && !inputs.auto_loiter;
  return decision;
}

ExecutorFlightState ExecutorLifecycle::state() const
{
  return state_;
}

VoxelPlanner::VoxelPlanner(PlannerConfig config)
: config_(config)
{
  if (config_.resolution <= 0.0 || config_.inflation_radius < 0.0 ||
    config_.max_expanded_voxels == 0 || config_.heuristic_weight < 1.0 ||
    config_.heuristic_weight > 3.0)
  {
    throw std::invalid_argument(
            "planner geometry, expansion budget, and heuristic weight are invalid");
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
  // An occupied endpoint can never be reached because occupied neighbors are
  // excluded below. Reject it before expanding the complete local volume.
  if (impl_->occupied(start) || impl_->occupied(goal)) {
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

  auto search = [&](bool restrict_vertical) {
      std::priority_queue<QueueEntry> open;
      std::unordered_map<GridIndex, double, GridIndexHash> g_score;
      std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
      std::unordered_set<GridIndex, GridIndexHash> closed;
      g_score[start_index] = 0.0;
      open.push(
        {config_.heuristic_weight * heuristic(start_index, goal_index), start_index});

      std::size_t expanded = 0;
      bool found = false;
      while (!open.empty() && expanded < config_.max_expanded_voxels) {
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
            const int minimum_dz = restrict_vertical ? 0 : -1;
            const int maximum_dz = restrict_vertical ? 0 : 1;
            for (int dz = minimum_dz; dz <= maximum_dz; ++dz) {
              if (dx == 0 && dy == 0 && dz == 0) {
                continue;
              }
              const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
              if (!in_bounds(neighbor) || occupied(neighbor) ||
                closed.find(neighbor) != closed.end())
              {
                continue;
              }
              const double step_cost = std::sqrt(
                static_cast<double>(dx * dx + dy * dy + dz * dz));
              const double tentative = g_score[current] + step_cost;
              const auto known = g_score.find(neighbor);
              if (known == g_score.end() || tentative < known->second) {
                g_score[neighbor] = tentative;
                came_from[neighbor] = current;
                open.push(
                  {tentative + config_.heuristic_weight * heuristic(neighbor, goal_index),
                    neighbor});
              }
            }
          }
        }
      }
      if (!found) {
        return std::vector<GridIndex>{};
      }

      std::vector<GridIndex> indices;
      GridIndex current = goal_index;
      indices.push_back(current);
      while (!(current == start_index)) {
        const auto parent = came_from.find(current);
        if (parent == came_from.end()) {
          return std::vector<GridIndex>{};
        }
        current = parent->second;
        indices.push_back(current);
      }
      std::reverse(indices.begin(), indices.end());
      return indices;
    };

  std::vector<GridIndex> grid_indices;
  if (start_index.z == goal_index.z) {
    grid_indices = search(true);
  }
  if (grid_indices.empty()) {
    grid_indices = search(false);
  }
  if (grid_indices.empty()) {
    return {};
  }

  std::vector<Vec3> grid_path;
  grid_path.reserve(grid_indices.size());
  for (std::size_t index = 0; index < grid_indices.size(); ++index) {
    if (index == 0) {
      grid_path.push_back(start);
    } else if (index + 1 == grid_indices.size()) {
      grid_path.push_back(goal);
    } else {
      grid_path.push_back(fromGrid(grid_indices[index], config_.resolution));
    }
  }

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

std::vector<TimedTrajectoryState> sampleCollisionFreePolyline(
  const std::vector<Vec3> & waypoints,
  double sample_period_seconds,
  double maximum_speed,
  double maximum_acceleration)
{
  if (!std::isfinite(sample_period_seconds) || sample_period_seconds <= 0.0 ||
    !std::isfinite(maximum_speed) || maximum_speed <= 0.0 ||
    !std::isfinite(maximum_acceleration) || maximum_acceleration <= 0.0)
  {
    throw std::invalid_argument("polyline sampling limits must be finite and positive");
  }
  if (waypoints.empty()) {
    return {};
  }

  std::vector<TimedTrajectoryState> samples;
  samples.push_back({0.0, {waypoints.front(), {}, {}, 0.0}});
  double elapsed = 0.0;
  for (std::size_t index = 1; index < waypoints.size(); ++index) {
    const Vec3 delta = waypoints[index] - waypoints[index - 1];
    const double length = norm(delta);
    if (!std::isfinite(length)) {
      throw std::invalid_argument("polyline waypoints must be finite");
    }
    if (length <= 1e-9) {
      continue;
    }
    const Vec3 direction = delta * (1.0 / length);
    double acceleration_duration = maximum_speed / maximum_acceleration;
    double acceleration_distance =
      0.5 * maximum_acceleration * acceleration_duration * acceleration_duration;
    double cruise_duration = 0.0;
    double peak_speed = maximum_speed;
    if (2.0 * acceleration_distance > length) {
      acceleration_duration = std::sqrt(length / maximum_acceleration);
      acceleration_distance =
        0.5 * maximum_acceleration * acceleration_duration * acceleration_duration;
      peak_speed = maximum_acceleration * acceleration_duration;
    } else {
      cruise_duration =
        (length - 2.0 * acceleration_distance) / maximum_speed;
    }
    const double segment_duration =
      2.0 * acceleration_duration + cruise_duration;
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(segment_duration / sample_period_seconds)));
    for (std::size_t step = 1; step <= steps; ++step) {
      const double time =
        segment_duration * static_cast<double>(step) / static_cast<double>(steps);
      double travelled = 0.0;
      double speed = 0.0;
      double acceleration = 0.0;
      if (step == steps) {
        travelled = length;
      } else if (time < acceleration_duration) {
        travelled = 0.5 * maximum_acceleration * time * time;
        speed = maximum_acceleration * time;
        acceleration = maximum_acceleration;
      } else if (time < acceleration_duration + cruise_duration) {
        const double cruise_time = time - acceleration_duration;
        travelled = acceleration_distance + peak_speed * cruise_time;
        speed = peak_speed;
      } else {
        const double remaining = segment_duration - time;
        travelled = length - 0.5 * maximum_acceleration * remaining * remaining;
        speed = maximum_acceleration * remaining;
        acceleration = -maximum_acceleration;
      }
      TrajectoryState state;
      state.position = waypoints[index - 1] + direction * travelled;
      state.velocity = direction * speed;
      state.acceleration = direction * acceleration;
      state.yaw = std::atan2(direction.y, direction.x);
      samples.push_back({
        elapsed + time,
        state,
      });
    }
    elapsed += segment_duration;
  }
  if (samples.size() == 1U) {
    samples.front().state.velocity = {};
    return samples;
  }
  samples.back().state.position = waypoints.back();
  samples.back().state.velocity = {};
  samples.back().state.acceleration = {};
  return samples;
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
    phase_ = FlightPhase::LAND;
    decision.phase = phase_;
    decision.request_land = true;
    decision.reason = "PX4 failsafe active";
    return decision;
  }

  if ((airborne_phase || phase_ == FlightPhase::HOLD) &&
    worst_data_age > inputs.land_timeout_seconds)
  {
    phase_ = FlightPhase::LAND;
    decision.phase = phase_;
    decision.request_land = true;
    decision.reason = "odometry or point cloud missing for more than 1 second";
    return decision;
  }
  if (phase_ == FlightPhase::HOLD &&
    worst_data_age <= inputs.hold_timeout_seconds)
  {
    phase_ = resume_phase_;
    decision.phase = phase_;
    decision.reason = "navigation inputs recovered";
    return decision;
  }
  if (airborne_phase && worst_data_age > inputs.hold_timeout_seconds) {
    resume_phase_ = phase_;
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
      if (!inputs.armed && !inputs.px4_ready) {
        phase_ = FlightPhase::PREFLIGHT;
        decision.reason = "prearm readiness lost during Offboard prestream";
        break;
      }
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
      if (!inputs.target_visible) {
        phase_ = FlightPhase::TARGET_SEARCH;
      } else if (inputs.target_aligned) {
        phase_ = FlightPhase::DROP_HOLD;
      }
      break;
    case FlightPhase::DROP_HOLD:
      decision.command_open_bottom_door = true;
      if (inputs.payload_released && inputs.drop_release_settled) {
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
      if (inputs.landed && !inputs.armed) {
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
