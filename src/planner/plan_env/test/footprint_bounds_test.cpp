#include "plan_env/footprint_bounds.h"
#include <limits>
#include <iostream>
void require(bool ok) { if (!ok) throw std::runtime_error("footprint bound regression"); }
int main() {
  const double pi = std::acos(-1.0);
  // Dense body samples, rotations, both signs of an off-centre sensor, and
  // the entire required margin must stay inside the collision envelope.
  for (double ox : {-0.2, 0.0, 0.2}) for (double oy : {-0.1, 0.0, 0.1}) {
    double radius = plan_env::circumscribedFootprintRadius(.82,.43,ox,oy,.30,.05);
    for (int a=0;a<72;++a) for (int i=0;i<=20;++i) for (int j=0;j<=20;++j) {
      double yaw=a*pi/36, x=ox-.41+.82*i/20, y=oy-.215+.43*j/20;
      double wx=std::cos(yaw)*x-std::sin(yaw)*y;
      double wy=std::sin(yaw)*x+std::cos(yaw)*y;
      require(std::hypot(wx,wy)+.30+std::sqrt(2.)*.05 <= radius+1e-12);
    }
  }
  require(std::abs(plan_env::circumscribedFootprintRadius(.82,.43,0,0,.30,.05)-
                   (std::hypot(.41,.215)+.30+std::sqrt(2.)*.05))<1e-12);
  for (double length : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected=false;
    try { plan_env::circumscribedFootprintRadius(length,.43,0,0,.3,.05); }
    catch (const std::invalid_argument&) { rejected=true; }
    require(rejected);
  }
  std::cout << "FOOTPRINT_BOUNDS_PASS\n";
}
