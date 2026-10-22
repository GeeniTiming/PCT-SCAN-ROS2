#pragma once
#include <cmath>
#include <cstddef>
#include <vector>

namespace scan_planner
{
inline bool canReuseTrajectory(double elapsed, double duration)
{
  return std::isfinite(elapsed) && std::isfinite(duration) &&
         elapsed >= 0.0 && duration > 0.0 && elapsed < duration;
}

inline bool hasUsableArcLength(const std::vector<double> &arc, std::size_t point_count)
{
  if (arc.size() < 2 || arc.size() != point_count || arc.front() != 0.0)
    return false;
  for (std::size_t i = 0; i < arc.size(); ++i)
    if (!std::isfinite(arc[i]) || (i > 0 && arc[i] < arc[i - 1]))
      return false;
  return arc.back() > 1e-6;
}
}  // namespace scan_planner
