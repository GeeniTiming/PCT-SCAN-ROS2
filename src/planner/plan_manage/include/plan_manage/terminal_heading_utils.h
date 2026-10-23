#pragma once
#include <cmath>
namespace scan_planner {
inline bool terminalYawReached(double target, double current, double tolerance) {
  return std::isfinite(target) && std::isfinite(current) && std::isfinite(tolerance) &&
      tolerance > 0.0 && std::abs(std::remainder(target-current, 2.0*std::acos(-1.0))) <= tolerance;
}
}
