#pragma once
#include "racecar_multipoint_controller/preview.hpp"
#include <limits>

namespace racecar_multipoint_controller {
// Ordered arc-length progress prevents switching branches at a figure-eight crossing.
class IndexedPath {
public:
  std::vector<Point> points;
  std::vector<double> distances;
  double progress=0.0;
  void reset(const std::vector<Point> &input,double spacing) {
    if (!std::isfinite(spacing) || spacing<=0 || input.size()<2)
      throw std::invalid_argument("Invalid indexed reference");
    std::vector<Point> clean;
    std::vector<double> arc;
    for (const auto &p:input) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("Nonfinite path");
      if (clean.empty()) {clean.push_back(p);arc.push_back(0);}
      else if (distance(clean.back(),p)>1e-9) {
        arc.push_back(arc.back()+distance(clean.back(),p));clean.push_back(p);
      }
    }
    if (clean.size()<2) throw std::invalid_argument("Reference has no length");
    points.clear();distances.clear();progress=0;
    std::size_t j=1;
    for (std::size_t i=0;;++i) {
      const double s=static_cast<double>(i)*spacing;
      if (s>=arc.back()-1e-9) break;
      while (j+1<arc.size() && arc[j]<s) ++j;
      const double t=(s-arc[j-1])/(arc[j]-arc[j-1]);
      points.push_back({clean[j-1].x+t*(clean[j].x-clean[j-1].x),
                        clean[j-1].y+t*(clean[j].y-clean[j-1].y)});
      distances.push_back(s);
    }
    points.push_back(clean.back());distances.push_back(arc.back());
  }
  void update(Point robot,double search_ahead) {
    if (points.size()<2 || !std::isfinite(search_ahead) || search_ahead<=0)
      throw std::invalid_argument("Invalid progress search");
    const double begin=std::max(0.0,progress-0.05);
    const double end=std::min(distances.back(),progress+search_ahead);
    auto it=std::upper_bound(distances.begin(),distances.end(),begin);
    std::size_t first=std::max<std::size_t>(1,it-distances.begin());
    double best=std::numeric_limits<double>::infinity(),best_s=progress;
    for (std::size_t j=first;j<points.size() && distances[j-1]<=end;++j) {
      const auto a=points[j-1],b=points[j];
      const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
      const double lo=std::max(0.0,(begin-distances[j-1])/(distances[j]-distances[j-1]));
      const double hi=std::min(1.0,(end-distances[j-1])/(distances[j]-distances[j-1]));
      if (hi<lo || d2<1e-16) continue;
      const double t=std::clamp(((robot.x-a.x)*dx+(robot.y-a.y)*dy)/d2,lo,hi);
      const double error=std::hypot(robot.x-a.x-t*dx,robot.y-a.y-t*dy);
      // Ties retain the earlier branch/point.
      if (error<best-1e-9) {best=error;best_s=distances[j-1]+t*(distances[j]-distances[j-1]);}
    }
    progress=std::max(progress,best_s);
  }
  std::size_t next_index() const {
    return std::min<std::size_t>(points.size()-1,
      std::upper_bound(distances.begin(),distances.end(),progress+1e-8)-distances.begin());
  }
};
inline Preview fit_indexed_points(const std::vector<Point>& points) {
  if (points.empty()) throw std::invalid_argument("Empty indexed preview");
  Preview out;out.points=points;
  double num=0,den=0;
  for (auto p:points) {const double r2=p.x*p.x+p.y*p.y;num+=2*r2*p.y;den+=r2*r2;}
  if (den>1e-10) out.curvature=num/den;
  if (!std::isfinite(out.curvature)) throw std::invalid_argument("Nonfinite indexed curvature");
  return out;
}
// Three-point circumcircle on metric chords; keeps actual extended bends while
// avoiding curvature spikes from short interpolation/quantization segments.
inline double planned_curvature(const IndexedPath &path,double lookahead,
                                double spacing,double sample_spacing) {
  const auto stride=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(sample_spacing/spacing)));
  const auto &p=path.points;
  double maximum=0.0;
  for (std::size_t i=std::max(path.next_index(),stride);
       i+stride<p.size() && path.distances[i]<=path.progress+lookahead;
       i+=std::max<std::size_t>(1,stride/2)) {
    const auto a=p[i-stride],b=p[i],c=p[i+stride];
    const double ab=std::hypot(b.x-a.x,b.y-a.y),bc=std::hypot(c.x-b.x,c.y-b.y);
    const double ac=std::hypot(c.x-a.x,c.y-a.y),den=ab*bc*ac;
    if (den>1e-9) maximum=std::max(maximum,
      2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/den);
  }
  return maximum;
}

}
