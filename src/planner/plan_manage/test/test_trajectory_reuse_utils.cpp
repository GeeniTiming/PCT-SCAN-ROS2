#include <plan_manage/trajectory_reuse_utils.h>
#include <limits>
#include <iostream>

int main()
{
  using namespace scan_planner;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  bool ok = canReuseTrajectory(0.0, 1.0) && canReuseTrajectory(0.999, 1.0);
  ok = ok && !canReuseTrajectory(1.0, 1.0) && !canReuseTrajectory(1.2, 1.0);
  ok = ok && !canReuseTrajectory(-0.1, 1.0) && !canReuseTrajectory(0.0, 0.0);
  ok = ok && !canReuseTrajectory(nan, 1.0) && !canReuseTrajectory(0.0, inf);
  ok = ok && !hasUsableArcLength({}, 0) && !hasUsableArcLength({0.0}, 0);
  ok = ok && !hasUsableArcLength({0.0}, 1) && !hasUsableArcLength({0.0, 0.0}, 2);
  ok = ok && !hasUsableArcLength({0.0, 0.1}, 1);
  ok = ok && !hasUsableArcLength({0.0, nan}, 2);
  ok = ok && !hasUsableArcLength({0.0, 0.2, 0.1}, 3);
  ok = ok && hasUsableArcLength({0.0, 0.0, 0.2}, 3);
  ok = ok && hasUsableArcLength({0.0, 0.1, 0.2}, 3);
  if (!ok) { std::cerr << "trajectory reuse regression FAILED\n"; return 1; }
  std::cout << "trajectory reuse regression PASS: expired, empty, singleton, degenerate, invalid and valid cases\n";
}
