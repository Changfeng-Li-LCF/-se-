#pragma once
#include <algorithm>
#include <cmath>
#include "racecar_multipoint_controller/indexed_path.hpp"

namespace racecar_multipoint_controller {
// Keep the distant speed preview separate from the near steering preview.
// A curve beyond entry_distance must not pull the target back onto the straight.
// Use the same metric curvature chords as planned_curvature, weighted by distance
// from ordered vehicle progress: full weight in the near half, smoothly fading
// to zero at entry_distance. The front and back chord endpoints are only used to
// estimate curvature; the sample center determines the entry weight.
inline double bend_entry_curvature(const IndexedPath &path, double spacing,
    double sample_spacing, double entry_distance) {
  const auto stride=std::max<std::size_t>(1,
    static_cast<std::size_t>(std::ceil(sample_spacing/spacing)));
  const auto &p=path.points;
  double maximum=0.0;
  for (std::size_t i=std::max(path.next_index(),stride);
       i+stride<p.size() && path.distances[i]<path.progress+entry_distance;
       i+=std::max<std::size_t>(1,stride/2)) {
    const double ahead=std::max(0.0,path.distances[i]-path.progress);
    const double transition=std::clamp(
      (ahead-0.5*entry_distance)/(0.5*entry_distance),0.0,1.0);
    const double weight=1.0-transition*transition*(3.0-2.0*transition);
    const auto a=p[i-stride],b=p[i],c=p[i+stride];
    const double ab=std::hypot(b.x-a.x,b.y-a.y),bc=std::hypot(c.x-b.x,c.y-b.y);
    const double ac=std::hypot(c.x-a.x,c.y-a.y),den=ab*bc*ac;
    if (den>1e-9) maximum=std::max(maximum,
      weight*2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/den);
  }
  return maximum;
}

// Radius bounds and curvature gain are unchanged.
inline double curvature_lookahead_target(double curvature, double minimum,
    double maximum, double gain_m, double deadband_per_m) {
  const double bend=std::max(0.0,std::abs(curvature)-deadband_per_m);
  return std::clamp(maximum/(1.0+gain_m*bend),minimum,maximum);
}

inline double smooth_curvature_lookahead(double current, double target,
    double dt_s, double tau_s, double minimum, double maximum) {
  const double alpha=-std::expm1(-dt_s/tau_s);
  const double next=current+alpha*(target-current);
  // Settle exactly at the straight-line radius instead of an asymptotic tail.
  return std::clamp(std::abs(next-target)<1e-4 ? target : next,minimum,maximum);
}
}  // namespace racecar_multipoint_controller
