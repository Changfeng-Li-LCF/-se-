#pragma once
#include "racecar_multipoint_controller/handoff_costmap.hpp"

namespace racecar_multipoint_controller {
// Sample the commanded motion in distance, without advancing a reference cursor.
inline std::vector<CurveState> motion_arc(Point robot,double yaw,double curvature,
    double horizon) {
  const auto count=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(horizon/0.10)));
  std::vector<CurveState> out;out.reserve(count+1);
  for (std::size_t i=0;i<=count;++i) {
    const double s=horizon*static_cast<double>(i)/count;
    const double angle=curvature*s;
    const Point p=std::abs(curvature)<1e-6 ? Point{s,0} :
      Point{std::sin(angle)/curvature,(1-std::cos(angle))/curvature};
    out.push_back({preview_world(p,robot,yaw),yaw+angle,curvature});
  }
  return out;
}

// Physical/unknown cells and map coverage define spaciousness. Inflation is
// still scored by the existing handoff validator; it is not a hard margin here.
// A conservative body circle is used ONLY to weaken this optional preference.
inline double motion_space_weight(const HandoffGrid &grid,
    const std::vector<CurveState> &arc,double body_radius,double lookahead) {
  if (!grid.data || grid.resolution<=0 || arc.empty()) return 0;
  const double close_margin=0.10*lookahead,wide_margin=0.45*lookahead;
  const double reach=body_radius+wide_margin;
  double clearance=reach;
  const double cell_radius=grid.resolution*std::sqrt(0.5);
  for (const auto &state:arc) {
    const auto p=state.point;
    clearance=std::min({clearance,p.x-grid.origin_x,p.y-grid.origin_y,
      grid.origin_x+grid.width*grid.resolution-p.x,
      grid.origin_y+grid.height*grid.resolution-p.y});
    if (clearance<=body_radius+close_margin) return 0;
    const int x0=std::max(0,static_cast<int>(std::floor((p.x-reach-grid.origin_x)/grid.resolution)));
    const int x1=std::min(static_cast<int>(grid.width)-1,
      static_cast<int>(std::floor((p.x+reach-grid.origin_x)/grid.resolution)));
    const int y0=std::max(0,static_cast<int>(std::floor((p.y-reach-grid.origin_y)/grid.resolution)));
    const int y1=std::min(static_cast<int>(grid.height)-1,
      static_cast<int>(std::floor((p.y+reach-grid.origin_y)/grid.resolution)));
    for (int y=y0;y<=y1;++y) for (int x=x0;x<=x1;++x) {
      if (grid.data[static_cast<std::size_t>(y)*grid.width+x]<254) continue;
      const Point cell{grid.origin_x+(x+0.5)*grid.resolution,
        grid.origin_y+(y+0.5)*grid.resolution};
      clearance=std::min(clearance,distance(p,cell)-cell_radius);
      if (clearance<=body_radius+close_margin) return 0;
    }
  }
  return std::clamp((clearance-body_radius-close_margin)/(wide_margin-close_margin),0.0,1.0);
}

// Persistent control state is the previous APPLIED curvature, not an old path
// or a timer restarted by replanning. The nonzero new-reference contribution
// always converges. Restricted space returns the original command immediately.
inline double continuous_motion_curvature(double previous,double requested,
    double dt,double lookahead,double speed,double openness,double minimum_tau) {
  if (openness<=0 || dt<=0) return requested;
  (void)lookahead;(void)speed;
  const double tau=minimum_tau;
  const double retained=std::exp(-dt/(std::clamp(openness,0.0,1.0)*tau));
  return requested+retained*(previous-requested);
}
// Once the old motion conflicts with the authoritative route, release its
// influence in a finite window. Replans do not restart this window. Vehicle/route
// changes still enter through the current reference on every control cycle.
class GoalPriorityBlend {
public:
  void clear() {active_=false;elapsed_=start_=0;}
  bool active() const {return active_;}
  double update(double candidate,double reference,double previous,double dt,
      double duration,double maximum_offset,double sign_epsilon) {
    const double error=candidate-reference;
    const bool opposing=candidate*reference<0 && std::abs(error)>sign_epsilon;
    if (active_) {
      if (!opposing && std::abs(error)<=0.5*maximum_offset) clear();
    } else if (opposing || std::abs(error)>maximum_offset) {
      active_=true;elapsed_=0;start_=previous;
    }
    if (!active_) return candidate;
    elapsed_+=std::max(0.0,dt);
    const double fraction=std::clamp(elapsed_/std::max(duration,1e-6),0.0,1.0);
    // Smoothstep reaches the NEW route exactly; no asymptotic old-turn tail.
    const double retained=1-fraction*fraction*(3-2*fraction);
    return retained*start_+(1-retained)*reference;
  }
private:
  bool active_=false;
  double elapsed_=0,start_=0;
};
} // namespace racecar_multipoint_controller
