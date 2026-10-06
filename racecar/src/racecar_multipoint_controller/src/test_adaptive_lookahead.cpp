#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "racecar_multipoint_controller/ordered_single_preview.hpp"
#include <iostream>
#include <stdexcept>
using namespace racecar_multipoint_controller;
class Probe:public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
 Probe(){use_velocity_scaled_lookahead_dist_=true;lookahead_time_=1.5;min_lookahead_dist_=.3;max_lookahead_dist_=.9;lookahead_dist_=.85;}
 double distance(double v){geometry_msgs::msg::Twist speed;speed.linear.x=v;return getLookAheadDistance(speed);}
 void fixed(){use_velocity_scaled_lookahead_dist_=false;}
};
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
int main(){
 Probe p;
 for(auto pair:std::vector<std::pair<double,double>>{{0,.3},{.1,.3},{.35,.525},{.5,.75},{1,.9},{-.35,.525}})
  require(std::abs(p.distance(pair.first)-pair.second)<1e-9,"Official clamp returned wrong value");
 IndexedPath path;path.reset({{0,0},{3,0}},.01);
 auto far=ordered_single_preview(path,{0,0},0,p.distance(1),0);
 auto near=ordered_single_preview(path,{0,0},0,p.distance(0),preview_search_floor(true,path.progress,far.progress));
 require(std::abs(near.point.x-.3)<1e-8,"Deceleration did not shorten target");
 require(near.progress<far.progress,"Previous target locks adaptive horizon");
 path.update({.5,0},.5);
 auto progressed=ordered_single_preview(path,{.5,0},0,p.distance(0),preview_search_floor(true,path.progress,far.progress));
 require(progressed.progress>=path.progress,"Adaptive target moved behind vehicle progress");
 p.fixed();require(std::abs(p.distance(0)-.85)<1e-9,"Fixed mode changed");
 require(preview_search_floor(false,.2,.8)==.8,"Legacy fixed progress lock changed");
 std::cout<<"PASS official speed clamp, reverse magnitude, stopped minimum, shrinking horizon, monotonic vehicle progress, fixed-mode fallback; no ROS graph or vehicle IO\n";
}
