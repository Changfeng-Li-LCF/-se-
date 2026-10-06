#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>
namespace racecar_multipoint_controller {
struct Point { double x, y; };
struct Preview {
  std::vector<Point> points;
  std::vector<double> radii;
  double curvature=0.0;
  bool endpoint_fallback=false;
};
inline double distance(Point a,Point b) {return std::hypot(a.x-b.x,a.y-b.y);}
// For each uniformly spaced radius, intersect the ordered reference polyline
// with the circle centered at the vehicle origin. Pick its first forward
// intersection at or after the previously selected path position. Input has
// already been pruned to vehicle progress and transformed to the base frame.
inline Preview preview(const std::vector<Point> &path,double inner,double outer,
                       std::size_t count) {
  if(!std::isfinite(inner) || !std::isfinite(outer) || inner<=0 || outer<=inner ||
     count<2 || count>200 || path.empty()) throw std::invalid_argument("Invalid circle preview configuration/path");
  std::vector<Point> vertices;vertices.reserve(path.size());
  for(const auto &p:path) {
    if(!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("Nonfinite reference point");
    if(vertices.empty() || distance(vertices.back(),p)>1e-9) vertices.push_back(p);
  }
  Preview out;out.points.reserve(count);out.radii.reserve(count);
  std::size_t cursor=1;double previous_t=0.0;
  for(std::size_t i=0;i<count;++i) {
    const double radius=inner+(outer-inner)*static_cast<double>(i)/(count-1);
    bool found=false;
    for(std::size_t j=cursor;j<vertices.size() && !found;++j) {
      const Point a=vertices[j-1],d{vertices[j].x-a.x,vertices[j].y-a.y};
      const double aa=d.x*d.x+d.y*d.y,b=a.x*d.x+a.y*d.y;
      const double c=a.x*a.x+a.y*a.y-radius*radius;
      const double disc=b*b-aa*c;
      if(disc<0.0) continue;
      const double root=std::sqrt(disc);
      // Sorted roots select the first eligible intersection along this segment.
      for(double t:{(-b-root)/aa,(-b+root)/aa}) {
        if(t<-1e-10 || t>1.0+1e-10 || (j==cursor && t<previous_t-1e-10)) continue;
        t=std::clamp(t,0.0,1.0);const Point q{a.x+t*d.x,a.y+t*d.y};
        if(q.x<-1e-10) continue;
        out.points.push_back(q);out.radii.push_back(radius);
        cursor=j;previous_t=t;found=true;break;
      }
    }
    // Missing intersections remain missing; do not invent points or repeat an
    // endpoint under the guise of 20 different circle intersections.
  }
  if(out.points.empty()) {
    const auto end=vertices.back();const double r=std::hypot(end.x,end.y);
    if(end.x>=0.0 && r<=outer) {out.points.push_back(end);out.endpoint_fallback=true;}
    else throw std::invalid_argument("No forward reference intersections for preview circles");
  }
  // Least-squares circle tangent to the vehicle axis: k*(x*x+y*y)=2*y.
  double num=0,den=0;
  for(auto p:out.points) {const double r2=p.x*p.x+p.y*p.y;num+=2*r2*p.y;den+=r2*r2;}
  if(den>1e-6*out.points.size()) out.curvature=num/den;
  if(!std::isfinite(out.curvature)) throw std::invalid_argument("Invalid fitted curvature");
  return out;
}
}  // namespace racecar_multipoint_controller
