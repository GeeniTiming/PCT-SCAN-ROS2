#pragma once
#include <cmath>
namespace scan_planner {
struct ArrivalPolicy {
  double since = -1.0;
  bool near(bool fresh, double distance) const {
    return fresh && std::isfinite(distance) && distance >= 0.0 && distance < 0.20;
  }
  bool settled(bool fresh, double distance, double speed, double now) {
    if (!near(fresh, distance) || !std::isfinite(speed) || speed < 0 || speed >= 0.05) {
      since = -1; return false;
    }
    if (since < 0 || now < since) since = now;
    return now - since >= 0.5;
  }
  void reset() { since = -1; }
};
}
