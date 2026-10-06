#pragma once
#include "racecar_multipoint_controller/indexed_path.hpp"

namespace racecar_multipoint_controller {
struct SinglePreview {
  Point point; // Vehicle frame: x forward, y left.
  std::size_t segment=0; // Target is between IDs segment and segment+1.
  double fraction=0.0,progress=0.0,curvature=0.0;
  const char *selection="circle";
};

// A shorter adaptive horizon must be allowed to move its target closer while
// actual vehicle progress remains monotonic. Fixed-horizon behavior is unchanged.
inline double preview_search_floor(bool adaptive,double vehicle_progress,double previous_target) {
  return adaptive ? vehicle_progress : previous_target;
}

// Use path IDs to disambiguate crossings, not a global nearest intersection.
// Progress and the previous target both constrain the start of the search.
// The search never extends beyond 2*radius of forward reference arc length.
inline SinglePreview ordered_single_preview(const IndexedPath &path,Point robot,
    double yaw,double radius,double previous_target_progress=0.0) {
  if (path.points.size()<2 || !std::isfinite(robot.x) || !std::isfinite(robot.y) ||
      !std::isfinite(yaw) || !std::isfinite(radius) || radius<=0 ||
      !std::isfinite(previous_target_progress) || previous_target_progress<0)
    throw std::invalid_argument("Invalid ordered single preview inputs");
  const double begin=std::max(path.progress,previous_target_progress);
  const double end=std::min(path.distances.back(),path.progress+2*radius);
  if (begin>end+1e-8) throw std::invalid_argument("Target progress outside local path window");
  const double c=std::cos(yaw),s=std::sin(yaw);
  auto base=[&](Point p) {
    const double x=p.x-robot.x,y=p.y-robot.y;
    return Point{c*x+s*y,-s*x+c*y};
  };
  auto result=[&](std::size_t j,double t,const char *mode) {
    const auto a=base(path.points[j-1]),b=base(path.points[j]);
    SinglePreview out;out.segment=j-1;out.fraction=t;out.selection=mode;
    out.progress=path.distances[j-1]+t*(path.distances[j]-path.distances[j-1]);
    out.point={a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
    const double r2=out.point.x*out.point.x+out.point.y*out.point.y;
    // Always use the actual target distance, including endpoint/fallback cases.
    out.curvature=r2>1e-10 ? 2*out.point.y/r2 : 0;
    return out;
  };
  const auto it=std::upper_bound(path.distances.begin(),path.distances.end(),begin);
  const std::size_t first=std::clamp<std::size_t>(it-path.distances.begin(),1,path.points.size()-1);
  for (std::size_t j=first;j<path.points.size() && path.distances[j-1]<=end;++j) {
    const double h=path.distances[j]-path.distances[j-1];
    const double lo=std::max(0.0,(begin-path.distances[j-1])/h);
    const double hi=std::min(1.0,(end-path.distances[j-1])/h);
    if (hi<lo) continue;
    const auto a=base(path.points[j-1]),b=base(path.points[j]);
    const double dx=b.x-a.x,dy=b.y-a.y,aa=dx*dx+dy*dy;
    const double dot=a.x*dx+a.y*dy,cc=a.x*a.x+a.y*a.y-radius*radius;
    const double discriminant=dot*dot-aa*cc;
    if (aa<1e-16 || discriminant < -1e-14) continue;
    const double root=std::sqrt(std::max(0.0,discriminant));
    for (double t:{(-dot-root)/aa,(-dot+root)/aa}) {
      if (t<lo-1e-9 || t>hi+1e-9) continue;
      auto out=result(j,std::clamp(t,lo,hi),"circle");
      if (out.point.x>=0) return out; // Earliest eligible ID, never another branch.
    }
  }
  auto at_progress=[&](double target,const char *mode) {
    const auto bound=std::upper_bound(path.distances.begin(),path.distances.end(),target);
    const auto j=std::clamp<std::size_t>(bound-path.distances.begin(),1,path.points.size()-1);
    const double t=std::clamp((target-path.distances[j-1])/(path.distances[j]-path.distances[j-1]),0.0,1.0);
    return result(j,t,mode);
  };
  // A nearby final point need not lie exactly on the lookahead circle.
  if (end==path.distances.back()) {
    auto out=at_progress(end,"endpoint");
    if (out.point.x>=-1e-8 && std::hypot(out.point.x,out.point.y)<=radius+1e-8) return out;
  }
  // If the car is farther than radius from the local path, no circle crossing
  // exists. Keep the same ordered branch and use an arc-length fallback.
  // Report the fallback instead of pretending its Euclidean distance is radius.
  const double desired=std::clamp(path.progress+radius,begin,end);
  auto fallback=at_progress(desired,"local_arc_fallback");
  if (fallback.point.x>1e-8) return fallback;
  for (std::size_t j=first;j<path.points.size() && path.distances[j]<=end;++j) {
    if (path.distances[j]<desired) continue;
    auto out=result(j,1.0,"local_arc_fallback");
    if (out.point.x>1e-8) return out;
  }
  throw std::invalid_argument("Ordered local reference is behind vehicle; no forward target");
}
} // namespace racecar_multipoint_controller
