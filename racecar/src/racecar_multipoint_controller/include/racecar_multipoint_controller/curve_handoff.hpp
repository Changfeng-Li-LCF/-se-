#pragma once
#include "racecar_multipoint_controller/preview_handoff.hpp"
#include <array>
#include <functional>
#include <optional>

namespace racecar_multipoint_controller {
struct CurveState {Point point;double yaw=0,curvature=0;};

inline Point path_point(const IndexedPath &path,double s) {
  s=std::clamp(s,0.0,path.distances.back());
  const auto bound=std::upper_bound(path.distances.begin(),path.distances.end(),s);
  const auto j=std::clamp<std::size_t>(bound-path.distances.begin(),1,path.points.size()-1);
  const double t=(s-path.distances[j-1])/(path.distances[j]-path.distances[j-1]);
  const auto a=path.points[j-1],b=path.points[j];
  return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
}
inline CurveState path_state(const IndexedPath &path,double s) {
  const double h=std::min(0.15,std::min(s,path.distances.back()-s));
  const auto p=path_point(path,s);
  if (h<1e-5) {
    const auto a=path_point(path,std::max(0.0,s-0.05));
    const auto b=path_point(path,std::min(path.distances.back(),s+0.05));
    return {p,std::atan2(b.y-a.y,b.x-a.x),0};
  }
  const auto a=path_point(path,s-h),b=path_point(path,s+h);
  const double den=distance(a,p)*distance(p,b)*distance(a,b);
  const double k=den>1e-9 ? 2*((p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x))/den : 0;
  return {p,std::atan2(b.y-a.y,b.x-a.x),k};
}

// Quintic Hermite interpolation fixes position, tangent and signed curvature
// at both ends. Validate the entire polynomial, not just its endpoints.
inline std::vector<CurveState> quintic_curve(CurveState a,CurveState b,
    double scale,double maximum_curvature) {
  const double chord=distance(a.point,b.point),m=chord*scale;
  if (chord<0.15 || std::abs(a.curvature)>maximum_curvature+1e-6 ||
      std::abs(b.curvature)>maximum_curvature+1e-6) return {};
  auto coefficients=[&](double p0,double p1,double d0,double d1,double dd0,double dd1) {
    std::array<double,6> q{p0,d0,dd0/2,0,0,0};
    const double A=p1-q[0]-q[1]-q[2],B=d1-q[1]-2*q[2],C=dd1-2*q[2];
    q[3]=10*A-4*B+C/2;q[4]=-15*A+7*B-C;q[5]=6*A-3*B+C/2;return q;
  };
  const auto x=coefficients(a.point.x,b.point.x,m*std::cos(a.yaw),m*std::cos(b.yaw),
    -a.curvature*m*m*std::sin(a.yaw),-b.curvature*m*m*std::sin(b.yaw));
  const auto y=coefficients(a.point.y,b.point.y,m*std::sin(a.yaw),m*std::sin(b.yaw),
    a.curvature*m*m*std::cos(a.yaw),b.curvature*m*m*std::cos(b.yaw));
  auto evaluate=[](const std::array<double,6> &q,double t) {
    return std::array<double,3>{
      q[0]+t*(q[1]+t*(q[2]+t*(q[3]+t*(q[4]+t*q[5])))),
      q[1]+t*(2*q[2]+t*(3*q[3]+t*(4*q[4]+t*5*q[5]))),
      2*q[2]+t*(6*q[3]+t*(12*q[4]+t*20*q[5]))};
  };
  const auto count=static_cast<std::size_t>(std::ceil(std::max(1.0,2*m)/0.02));
  std::vector<CurveState> out;out.reserve(count+1);
  for (std::size_t i=0;i<=count;++i) {
    const double t=static_cast<double>(i)/count;const auto px=evaluate(x,t),py=evaluate(y,t);
    const double d2=px[1]*px[1]+py[1]*py[1];
    if (d2<1e-8 || px[1]*(b.point.x-a.point.x)+py[1]*(b.point.y-a.point.y)<=0) return {};
    const double k=(px[1]*py[2]-py[1]*px[2])/std::pow(d2,1.5);
    if (!std::isfinite(k) || std::abs(k)>maximum_curvature+1e-6) return {};
    out.push_back({{px[0],py[0]},std::atan2(py[1],px[1]),k});
  }
  return out;
}

struct CurveHandoff {
  IndexedPath path;
  double target_floor=0,score=0;
  double kept_prefix_m=0,reference_error_m=0;
  bool retained_prefix=false;
};
using CurveValidator=std::function<std::optional<double>(const std::vector<CurveState>&,bool)>;

struct HandoffReference {
  SinglePreview old_target,new_target;
  double separation=0;
  bool opposing=false;
};
inline std::optional<HandoffReference> handoff_reference(const IndexedPath &old_path,
    const IndexedPath &new_path,Point robot,double yaw,double lookahead,
    double old_floor,double spacing) {
  HandoffReference out;
  try {
    out.old_target=ordered_single_preview(old_path,robot,yaw,lookahead,old_floor);
    out.new_target=ordered_single_preview(new_path,robot,yaw,lookahead,new_path.progress);
  } catch (const std::invalid_argument &) {return std::nullopt;}
  out.separation=distance(out.old_target.point,out.new_target.point);
  out.opposing=out.old_target.curvature*out.new_target.curvature<0 &&
    std::abs(out.old_target.curvature-out.new_target.curvature)>4*spacing/(lookahead*lookahead);
  return out;
}

// A bounded local splice, never a whole-route nearest-point search. The already
// accepted near prefix is only a short continuity preference. Candidate targets
// converge to the NEW ordered reference; old targets must not perpetuate a turn
// in the opposite direction. New obstacles can also invalidate that preference.
inline std::optional<CurveHandoff> make_curve_handoff(const IndexedPath &old_path,
    const IndexedPath &new_path,Point robot,double yaw,double lookahead,double old_floor,
    double executed_curvature,double minimum_radius,double spacing,double search_ahead,
    const CurveValidator &validate,double motion_preference=0,
    double goal_join_limit=std::numeric_limits<double>::infinity()) {
  if (old_path.points.size()<2 || new_path.points.size()<2 || minimum_radius<=0 ||
      std::min(new_path.distances.back(),goal_join_limit)-new_path.progress<1.3*lookahead) return std::nullopt;
  const auto reference=handoff_reference(old_path,new_path,robot,yaw,lookahead,old_floor,spacing);
  if (!reference) return std::nullopt;
  const auto old_target=reference->old_target;
  const auto old_world=preview_world(old_target.point,robot,yaw);
  const double ahead=old_target.progress-old_path.progress;
  const auto vehicle=project_local(new_path,robot,new_path.progress,
    new_path.progress+search_ahead);
  const auto mapped=project_local(new_path,old_world,
    new_path.progress+std::max(0.0,ahead-search_ahead),
    std::min(new_path.distances.back(),new_path.progress+std::min(2*lookahead,ahead+search_ahead)));
  if (!vehicle.valid || !mapped.valid || vehicle.error>search_ahead || mapped.error>search_ahead ||
      !same_forward_segment(old_path,old_path.next_index()-1,new_path,vehicle.segment) ||
      !same_forward_segment(old_path,old_target.segment,new_path,mapped.segment)) return std::nullopt;

  const double max_k=1/minimum_radius;
  struct Candidate {CurveHandoff result;std::vector<CurveState> curve;double join;};
  std::vector<Candidate> candidates;
  // Do not hold the entire old lookahead (previously >=1.0m). Release sooner
  // when the new target has moved or requires the opposite turn. The endpoint
  // remains fixed in path coordinates until traversed; no timer is restarted.
  const double release=reference->opposing ? 1.0 :
    std::clamp(reference->separation/(0.15*lookahead),0.0,1.0);
  const double continuity=1-release;
  const double keep_end=std::min(old_path.distances.back(),
    old_path.progress+0.25*lookahead*continuity);
  std::vector<CurveState> prefix;
  for (double s=old_path.progress;s<keep_end;s+=0.02) prefix.push_back(path_state(old_path,s));
  prefix.push_back(path_state(old_path,keep_end));
  const bool prefix_feasible=keep_end-old_path.progress>2*spacing &&
    std::all_of(prefix.begin(),prefix.end(),[&](const auto &p) {
    return std::abs(p.curvature)<=max_k+1e-6;
  });
  const double keep_ahead=keep_end-old_path.progress;
  for (bool retain:{true,false}) {
    if (retain && !prefix_feasible) continue;
    const CurveState start=retain ? prefix.back() : CurveState{robot,yaw,executed_curvature};
    for (double length:{1.55*lookahead,1.85*lookahead,2.0*lookahead}) {
      // Never replace the part containing an unpassed ordered goal.
      const double join=std::min({new_path.distances.back()-0.15,
        new_path.progress+length,goal_join_limit});
      if (join-new_path.progress<1.3*lookahead || (retain && length<keep_ahead+0.2)) continue;
      const auto end=path_state(new_path,join);
      for (double scale:{1.0,1.2}) {
        auto curve=quintic_curve(start,end,scale,max_k);
        if (curve.empty()) continue;
        if (retain) curve.insert(curve.begin(),prefix.begin(),prefix.end()-1);
        // Check only the new/retained near geometry; the untouched suffix is
        // still the planner's responsibility. Collision rejects a candidate,
        // not the navigation task.
        // Cheap center-cost scoring first. Body rasterization is performed in
        // score order and stops at the first safe candidate, avoiding twelve
        // full footprint scans on every high-frequency path replacement.
        const auto obstacle_cost=validate(curve,false);if (!obstacle_cost) continue;
        std::vector<Point> input;input.reserve(curve.size());
        for (const auto &p:curve) input.push_back(p.point);
        CurveHandoff candidate;candidate.path.reset(input,spacing);candidate.retained_prefix=retain;
        SinglePreview target;
        try {target=ordered_single_preview(candidate.path,robot,yaw,lookahead);}
        catch (const std::invalid_argument &) {continue;}
        double energy=0,variation=0,length_sum=0;
        for (std::size_t i=0;i<curve.size();++i) {
          energy+=curve[i].curvature*curve[i].curvature;
          if (i) {
            const double ds=std::max(0.01,distance(curve[i-1].point,curve[i].point));
            variation+=std::pow(curve[i].curvature-curve[i-1].curvature,2)/ds;
            length_sum+=ds;
          }
        }
        const double new_shift=distance(target.point,reference->new_target.point);
        const double old_shift=distance(target.point,old_target.point);
        const double preference=std::clamp(motion_preference,0.0,1.0);
        // The new ordered route is always the primary reference. Spaciousness
        // only changes the secondary smoothness preference, not goal attraction.
        candidate.score=10*new_shift*new_shift+
          0.5*std::pow(target.curvature-reference->new_target.curvature,2)+
          0.25*continuity*old_shift*old_shift+
          (0.1+0.2*continuity+0.25*preference)*std::pow(target.curvature-executed_curvature,2)+
          0.25*(*obstacle_cost)+0.03*energy/curve.size()+
          0.02*preference*variation/std::max(0.1,length_sum);
        candidate.target_floor=target.progress;
        candidate.kept_prefix_m=retain ? keep_ahead : 0;
        candidate.reference_error_m=new_shift;
        candidates.push_back({std::move(candidate),std::move(curve),join});
      }
    }
  }
  // Fixed error bands give a transitive ordering: first convergence toward the
  // new route, then obstacle cost and smoothness within the same small band.
  const double reference_band=std::max(2*spacing,0.05*lookahead);
  std::stable_sort(candidates.begin(),candidates.end(),[reference_band](const auto &a,const auto &b) {
    const auto ak=std::floor(a.result.reference_error_m/reference_band);
    const auto bk=std::floor(b.result.reference_error_m/reference_band);
    if (ak!=bk) return ak<bk;
    return a.result.score<b.result.score;
  });
  for (auto &candidate:candidates) {
    if (!validate(candidate.curve,true)) continue;
    // Resample the untouched full suffix only once, for the accepted candidate.
    std::vector<Point> input;input.reserve(candidate.curve.size()+new_path.points.size());
    for (const auto &p:candidate.curve) input.push_back(p.point);
    const auto tail=std::upper_bound(new_path.distances.begin(),new_path.distances.end(),candidate.join);
    input.insert(input.end(),new_path.points.begin()+(tail-new_path.distances.begin()),new_path.points.end());
    candidate.result.path.reset(input,spacing);
    return std::move(candidate.result);
  }
  return std::nullopt;
}
} // namespace racecar_multipoint_controller
