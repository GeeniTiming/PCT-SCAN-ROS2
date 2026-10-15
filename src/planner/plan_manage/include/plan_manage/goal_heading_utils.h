#ifndef PLAN_MANAGE_GOAL_HEADING_UTILS_H
#define PLAN_MANAGE_GOAL_HEADING_UTILS_H

#include <algorithm>
#include <cmath>
#include <optional>
#include <geometry_msgs/msg/quaternion.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>

namespace scan_planner
{
inline std::optional<double> terminalYawFromQuaternion(
    const geometry_msgs::msg::Quaternion &q)
{
  if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w))
    return std::nullopt;
  const double scale = std::max({std::abs(q.x), std::abs(q.y),
                                 std::abs(q.z), std::abs(q.w)});
  if (scale < 1e-12) return std::nullopt;
  const double sx = q.x / scale, sy = q.y / scale;
  const double sz = q.z / scale, sw = q.w / scale;
  const double norm = std::sqrt(sx*sx + sy*sy + sz*sz + sw*sw);
  const double x = sx/norm, y = sy/norm, z = sz/norm, w = sw/norm;
  // Identity is an explicit map +X heading, not an absent heading.
  return std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
}

inline void encodeTerminalGoalYaw(scan_planner_msgs::msg::Bspline &message,
                                  const std::optional<double> &yaw,
                                  double duration)
{
  message.yaw_pts.clear();
  message.yaw_dt = 0.0;
  if (!yaw || !std::isfinite(*yaw) || !std::isfinite(duration) || duration <= 0.0)
    return;
  // A constant two-sample profile carries the GLOBAL terminal heading.
  // Only a stationary position trajectory requests terminal alignment.
  const double normalized = std::atan2(std::sin(*yaw), std::cos(*yaw));
  message.yaw_pts = {normalized, normalized};
  message.yaw_dt = duration;
}

inline std::optional<double> decodeTerminalGoalYaw(
    const scan_planner_msgs::msg::Bspline &message)
{
  if (message.yaw_pts.size() < 2 || !std::isfinite(message.yaw_dt) ||
      message.yaw_dt <= 0.0)
    return std::nullopt;
  for (double yaw : message.yaw_pts)
    if (!std::isfinite(yaw)) return std::nullopt;
  const double yaw = message.yaw_pts.back();
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
inline bool isTerminalAlignmentTrajectory(const scan_planner_msgs::msg::Bspline &message)
{
  if (!decodeTerminalGoalYaw(message) || message.pos_pts.size() < 2)
    return false;
  const auto &first = message.pos_pts.front();
  for (const auto &point : message.pos_pts)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::hypot(point.x-first.x, point.y-first.y, point.z-first.z) > 1e-6)
      return false;
  }
  return true;
}

inline bool terminalHeadingReached(const std::optional<double> &goal,
                                   const std::optional<double> &current,
                                   double tolerance)
{
  if (!goal) return true; // Legacy position-only task.
  return current && std::isfinite(*goal) && std::isfinite(*current) &&
      std::abs(std::remainder(*goal-*current, 2.0*M_PI)) <= tolerance;
}

inline double terminalAlignmentYawRate(double error, double tolerance, double gain,
                                       double min_rate, double max_rate)
{
  if (!std::isfinite(error)) return 0.0;
  error = std::remainder(error, 2.0*M_PI);
  if (std::abs(error) <= tolerance) return 0.0;
  const double rate = std::clamp(gain*std::abs(error), min_rate, max_rate);
  return std::copysign(rate, error);
}
} // namespace scan_planner
#endif
