#pragma once
#include "racecar_multipoint_controller/curve_handoff.hpp"
#include <cstdint>

namespace racecar_multipoint_controller {
struct HandoffGrid {
  const unsigned char *data=nullptr;
  unsigned width=0,height=0;
  double resolution=0,origin_x=0,origin_y=0;
};

// Convex physical footprint versus cell squares (SAT), including the body
// interior. Padding/inflation is a soft cost, not an extra hard body margin.
inline bool footprint_overlaps_cell(const std::vector<Point> &polygon,
    double x,double y,double half) {
  for (std::size_t i=0;i<polygon.size();++i) {
    const auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
    const double nx=-(b.y-a.y),ny=b.x-a.x;
    double lo=std::numeric_limits<double>::infinity(),hi=-lo;
    for (const auto &p:polygon) {const double d=p.x*nx+p.y*ny;lo=std::min(lo,d);hi=std::max(hi,d);}
    const double center=x*nx+y*ny,radius=half*(std::abs(nx)+std::abs(ny));
    if (hi<center-radius-1e-9 || lo>center+radius+1e-9) return false;
  }
  return true;
}
inline std::optional<double> handoff_footprint_cost(const HandoffGrid &grid,
    const CurveState &state,const std::vector<Point> &footprint,bool allow_unknown,
    bool require_coverage) {
  if (!grid.data || grid.resolution<=0 || footprint.size()<3) return std::nullopt;
  std::vector<Point> polygon;polygon.reserve(footprint.size());
  double xmin=std::numeric_limits<double>::infinity(),ymin=xmin,xmax=-xmin,ymax=-xmin;
  for (auto p:footprint) {
    p=preview_world(p,state.point,state.yaw);polygon.push_back(p);
    xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);
  }
  const int x0=static_cast<int>(std::floor((xmin-grid.origin_x)/grid.resolution));
  const int x1=static_cast<int>(std::floor((xmax-grid.origin_x)/grid.resolution));
  const int y0=static_cast<int>(std::floor((ymin-grid.origin_y)/grid.resolution));
  const int y1=static_cast<int>(std::floor((ymax-grid.origin_y)/grid.resolution));
  if (require_coverage && (x0<0 || y0<0 || x1>=static_cast<int>(grid.width) || y1>=static_cast<int>(grid.height))) return std::nullopt;
  double total=0;std::size_t count=0;
  for (int y=std::max(0,y0);y<=std::min(y1,static_cast<int>(grid.height)-1);++y) {
    for (int x=std::max(0,x0);x<=std::min(x1,static_cast<int>(grid.width)-1);++x) {
      const double wx=grid.origin_x+(x+0.5)*grid.resolution,wy=grid.origin_y+(y+0.5)*grid.resolution;
      // The bounding cell may overlap only on its far side: include box axes.
      const double half=grid.resolution/2;
      if (wx+half<xmin || wx-half>xmax || wy+half<ymin || wy-half>ymax ||
          !footprint_overlaps_cell(polygon,wx,wy,half)) continue;
      const auto cost=grid.data[static_cast<std::size_t>(y)*grid.width+x];
      if (cost==254 || (cost==255 && !allow_unknown)) return std::nullopt;
      total+=cost==255 ? 1.0 : static_cast<double>(cost)/253; ++count;
    }
  }
  return count ? total/count : 0.0;
}
} // namespace racecar_multipoint_controller
