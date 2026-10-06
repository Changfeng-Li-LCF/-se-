#pragma once
#include "racecar_multipoint_controller/ordered_single_preview.hpp"
#include <limits>

namespace racecar_multipoint_controller {
struct LocalProjection {
  Point point{0,0};
  std::size_t segment=0;
  double progress=0,error=std::numeric_limits<double>::infinity();
  bool valid=false;
};

// Restrict correspondence to the current ordered prefix, including at crossings.
inline LocalProjection project_local(const IndexedPath &path,Point query,double begin,double end) {
  LocalProjection out;
  if (path.points.size()<2 || !std::isfinite(query.x) || !std::isfinite(query.y) || end<begin) return out;
  begin=std::max(0.0,begin);end=std::min(end,path.distances.back());
  const auto it=std::upper_bound(path.distances.begin(),path.distances.end(),begin);
  const auto first=std::clamp<std::size_t>(it-path.distances.begin(),1,path.points.size()-1);
  for (std::size_t j=first;j<path.points.size() && path.distances[j-1]<=end;++j) {
    const auto a=path.points[j-1],b=path.points[j];
    const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy,h=path.distances[j]-path.distances[j-1];
    const double lo=std::max(0.0,(begin-path.distances[j-1])/h);
    const double hi=std::min(1.0,(end-path.distances[j-1])/h);
    if (hi<lo || d2<1e-16) continue;
    const double t=std::clamp(((query.x-a.x)*dx+(query.y-a.y)*dy)/d2,lo,hi);
    const Point p{a.x+t*dx,a.y+t*dy};const double error=distance(query,p);
    if (error<out.error-1e-9) out={p,j-1,path.distances[j-1]+t*h,error,true};
  }
  return out;
}

inline bool same_forward_segment(const IndexedPath &a,std::size_t i,const IndexedPath &b,std::size_t j) {
  const Point da{a.points[i+1].x-a.points[i].x,a.points[i+1].y-a.points[i].y};
  const Point db{b.points[j+1].x-b.points[j].x,b.points[j+1].y-b.points[j].y};
  // Compatibility only: an incompatible reroute is followed directly, not stopped.
  return da.x*db.x+da.y*db.y>=std::sqrt(0.5)*std::hypot(da.x,da.y)*std::hypot(db.x,db.y);
}

inline bool same_geometry(const IndexedPath &a,const IndexedPath &b) {
  if (a.points.size()!=b.points.size() || a.points.empty()) return false;
  if (std::abs(a.distances.back()-b.distances.back())>1e-9) return false;
  for (std::size_t i=0;i<a.points.size();++i) if (distance(a.points[i],b.points[i])>1e-9) return false;
  return true;
}

inline Point preview_world(Point p,Point robot,double yaw) {
  const double c=std::cos(yaw),s=std::sin(yaw);
  return {robot.x+c*p.x-s*p.y,robot.y+s*p.x+c*p.y};
}
inline Point preview_body(Point p,Point robot,double yaw) {
  const double c=std::cos(yaw),s=std::sin(yaw),x=p.x-robot.x,y=p.y-robot.y;
  return {c*x+s*y,-s*x+c*y};
}

// Smooth only replacement-induced changes. Normal path/vehicle motion is not
// low-pass filtered. Repeated updates add their delta without restarting a timer.
class PreviewHandoff {
public:
  void clear() {offset_={0,0};}
  void decay(double dt,double tau) {
    const double weight=std::exp(-dt/tau);
    offset_.x*=weight;offset_.y*=weight;
    if (std::hypot(offset_.x,offset_.y)<1e-5) clear();
  }
  bool join(Point old_target,Point new_target,double maximum_shift) {
    const Point next{offset_.x+old_target.x-new_target.x,offset_.y+old_target.y-new_target.y};
    if (distance(old_target,new_target)>maximum_shift || std::hypot(next.x,next.y)>maximum_shift) {
      clear();return false;
    }
    offset_=next;return true;
  }
  Point apply(Point target) const {return {target.x+offset_.x,target.y+offset_.y};}
  double shift() const {return std::hypot(offset_.x,offset_.y);}
private:
  Point offset_{0,0};
};
} // namespace racecar_multipoint_controller
