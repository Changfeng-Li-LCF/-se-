#pragma once
#include <algorithm>
#include <cmath>

namespace racecar_multipoint_controller {
// Residual between references evaluated at a common pose. This is NOT a
// low-pass filter on vehicle feedback or the complete steering command.
class CurvatureHandoff {
public:
  void clear() {offset_=0;}
  void join(double old_curvature,double new_curvature) {
    offset_+=old_curvature-new_curvature;
  }
  double update(double curvature,double dt,double tau) {
    offset_*=std::exp(-std::max(0.0,dt)/tau);
    if (std::abs(offset_)<1e-5) clear();
    return curvature+offset_;
  }
  double offset() const {return offset_;}
private:
  double offset_=0;
};
} // namespace racecar_multipoint_controller
