#pragma once

#include <cmath>

namespace scan_planner {

// Arrival detection thresholds.
//
// Originally near() required distance < 0.20 m and settled() required
// speed < 0.05 m/s for 0.5 s.  With the lightning-based localisation on this
// robot the pose error/jitter during navigation is easily 0.2-0.5 m, so the
// 0.20 m threshold was frequently unreachable: C never declared arrival, kept
// re-planning and drove past the goal (and into whatever stood behind it).
// Relaxed to 0.45 m / 0.08 m/s / 0.4 s and the caller additionally treats
// "trajectory time expired while within 0.8 m of the goal" as arrival.
struct ArrivalPolicy {
  double since = -1.0;

  bool near(bool fresh, double distance) const {
    return fresh && std::isfinite(distance) && distance >= 0.0 && distance < 0.45;
  }

  bool settled(bool fresh, double distance, double speed, double now) {
    if (!near(fresh, distance) || !std::isfinite(speed) || speed < 0 || speed >= 0.08) {
      since = -1;
      return false;
    }
    if (since < 0 || now < since) since = now;
    return now - since >= 0.4;
  }

  void reset() { since = -1.0; }
};

}  // namespace scan_planner
