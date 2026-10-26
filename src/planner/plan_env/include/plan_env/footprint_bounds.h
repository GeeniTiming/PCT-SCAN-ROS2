#pragma once
#include <cmath>
#include <stdexcept>

namespace plan_env {
// A yaw-independent bound of a body rectangle about the odometry origin.
// Offset is body-centre minus odometry-origin, expressed in the body frame.
// The reference point stays unchanged: even an uncertain yaw cannot shrink it.
inline double circumscribedFootprintRadius(double length, double width,
    double offset_x, double offset_y, double margin, double resolution) {
  if (!std::isfinite(length) || !std::isfinite(width) ||
      !std::isfinite(offset_x) || !std::isfinite(offset_y) ||
      !std::isfinite(margin) || !std::isfinite(resolution) ||
      length <= 0 || width <= 0 || margin < 0 || resolution <= 0)
    throw std::invalid_argument("invalid footprint geometry or safety margin");
  // Farthest rectangle corner plus clearance. Two half-cell diagonals
  // cover query-cell snapping and the occupied cell's planar extent.
  return std::hypot(0.5 * length + std::abs(offset_x),
                    0.5 * width + std::abs(offset_y)) + margin +
         std::sqrt(2.0) * resolution;
}
}  // namespace plan_env
