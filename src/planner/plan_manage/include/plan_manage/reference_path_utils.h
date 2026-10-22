#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/path.hpp>

namespace scan_planner
{

inline std::vector<Eigen::Vector3d> simplifyReferenceWaypoints(
    const std::vector<Eigen::Vector3d> &points,
    double tolerance)
{
  if (points.size() <= 2 || tolerance <= 0.0)
    return points;

  std::vector<bool> keep(points.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<size_t, size_t>> ranges{{0, points.size() - 1}};

  while (!ranges.empty())
  {
    const auto [first, last] = ranges.back();
    ranges.pop_back();
    if (last <= first + 1)
      continue;

    const Eigen::Vector3d segment = points[last] - points[first];
    const double squared_length = segment.squaredNorm();
    double max_distance = -1.0;
    size_t max_index = first;

    for (size_t index = first + 1; index < last; ++index)
    {
      double distance;
      if (squared_length <= 1e-12)
      {
        distance = (points[index] - points[first]).norm();
      }
      else
      {
        const double ratio = std::clamp(
            (points[index] - points[first]).dot(segment) / squared_length,
            0.0, 1.0);
        const Eigen::Vector3d projection = points[first] + ratio * segment;
        distance = (points[index] - projection).norm();
      }

      if (distance > max_distance)
      {
        max_distance = distance;
        max_index = index;
      }
    }

    if (max_distance > tolerance)
    {
      keep[max_index] = true;
      ranges.emplace_back(first, max_index);
      ranges.emplace_back(max_index, last);
    }
  }

  std::vector<Eigen::Vector3d> simplified;
  simplified.reserve(points.size());
  for (size_t index = 0; index < points.size(); ++index)
  {
    if (keep[index])
      simplified.push_back(points[index]);
  }
  return simplified;
}

// PCT points describe the terrain; odometry describes the tracked sensor.
// Keep terrain elevation changes while anchoring the first point to odometry.
inline bool referenceHeightOffset(double terrain_start_z, double odom_z, double &offset)
{
  if (!std::isfinite(terrain_start_z) || !std::isfinite(odom_z))
    return false;
  offset = odom_z - terrain_start_z;
  return std::isfinite(offset);
}

inline bool prepareReferenceWaypoints(
    const nav_msgs::msg::Path &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr,
    double simplify_tolerance = 0.0,
    double *terminal_yaw = nullptr)
{
  waypoints.clear();
  if (path.poses.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0 ||
      !std::isfinite(simplify_tolerance) || simplify_tolerance < 0.0)
  {
    if (error)
      *error = "body height, minimum distance, and simplify tolerance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.poses.size());
  Eigen::Vector3d final_waypoint;
  Eigen::Vector3d last_waypoint;
  bool first = true;

  for (const auto &pose_stamped : path.poses)
  {
    const auto &position = pose_stamped.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    Eigen::Vector3d waypoint(position.x, position.y, position.z + body_height);
    final_waypoint = waypoint;
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }

  waypoints = simplifyReferenceWaypoints(waypoints, simplify_tolerance);

  if (terminal_yaw)
  {
    // The final pose orientation encodes the desired terminal heading.  An
    // identity (or absent) orientation carries no heading, so report NaN and
    // let the FSM fall back to the tangent behaviour.
    const auto &q = path.poses.back().pose.orientation;
    const double norm_xy = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    *terminal_yaw = (norm_xy > 1e-3)
        ? std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        : std::numeric_limits<double>::quiet_NaN();
  }

  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
