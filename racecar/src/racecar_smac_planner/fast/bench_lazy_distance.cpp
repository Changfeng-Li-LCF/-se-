#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include "nav2_smac_planner/node_hybrid.hpp"
#include "nav2_smac_planner/a_star.hpp"
// The vendored robin_hood header is identical and uses a global namespace.
#define RACECAR_SMAC_PLANNER__THIRDPARTY__ROBIN_HOOD_H_
#include "racecar_smac_planner/node_hybrid.hpp"
#include "racecar_smac_planner/a_star.hpp"

using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a){return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}

int main() {
  std::mt19937 rng(2313);unsigned int checks=0;double max_error=0;
  std::cout<<"{\"cases\":[";bool first=true;
  for (int model=0;model<2;++model) for (double radius:{16.,18.,24.}) {
    nav2_smac_planner::SearchInfo old_info{};
    racecar_smac_planner::SearchInfo new_info{};
    old_info.minimum_turning_radius=radius;new_info.minimum_turning_radius=radius;
    auto old_model=model ? nav2_smac_planner::MotionModel::REEDS_SHEPP : nav2_smac_planner::MotionModel::DUBIN;
    auto new_model=model ? racecar_smac_planner::MotionModel::REEDS_SHEPP : racecar_smac_planner::MotionModel::DUBIN;
    const float dimension=161.;unsigned int w=200,h=200,bins=64;
    auto t=Clock::now();
    nav2_smac_planner::NodeHybrid::precomputeDistanceHeuristic(dimension,old_model,bins,old_info);
    const double eager=ms(t);
    t=Clock::now();
    racecar_smac_planner::NodeHybrid::precomputeDistanceHeuristic(dimension,new_model,bins,new_info);
    const double lazy=ms(t);
    nav2_smac_planner::NodeHybrid::initMotionModel(old_model,w,h,bins,old_info);
    racecar_smac_planner::NodeHybrid::initMotionModel(new_model,w,h,bins,new_info);
    std::vector<std::array<float,6>> coordinates;
    for (int i=0;i<2000;++i) {
      coordinates.push_back({float(10+rng()%140),float(10+rng()%140),float(rng()%bins),
                             80.f,80.f,float(rng()%bins)});
    }
    // Include mirrored y and zero heading, which exercise upstream flat-index addressing.
    coordinates.push_back({81,79,0,80,80,0});
    t=Clock::now();
    for (const auto &c:coordinates) {
      auto a=nav2_smac_planner::NodeHybrid::getDistanceHeuristic({c[0],c[1],c[2]},{c[3],c[4],c[5]},1.f);
      auto b=racecar_smac_planner::NodeHybrid::getDistanceHeuristic({c[0],c[1],c[2]},{c[3],c[4],c[5]},1.f);
      max_error=std::max(max_error,std::abs(double(a-b)));checks++;
      if (!std::isfinite(b) || std::abs(a-b)>1e-4) {std::cerr<<"Distance mismatch "<<a<<" "<<b<<"\n";return 2;}
    }
    const double queries=ms(t);
    t=Clock::now();
    for (const auto &c:coordinates)
      racecar_smac_planner::NodeHybrid::getDistanceHeuristic({c[0],c[1],c[2]},{c[3],c[4],c[5]},1.f);
    const double warm=ms(t);
    // The actual radius-update path constructs and initializes a new A* object.
    t=Clock::now();
    racecar_smac_planner::AStarAlgorithm<racecar_smac_planner::NodeHybrid> algorithm(new_model,new_info);
    int iterations=100000;
    algorithm.initialize(false,iterations,1000,1.,dimension,bins);
    const double astar=ms(t);
    if(!first)std::cout<<",";first=false;
    std::cout<<"{\"model\":\""<<(model ? "REEDS_SHEPP" : "DUBIN")<<"\",\"radius_cells\":"<<radius
      <<",\"eager_init_ms\":"<<eager<<",\"lazy_init_ms\":"<<lazy
      <<",\"astar_reinit_ms\":"<<astar<<",\"cold_2001_queries_comparison_ms\":"<<queries
      <<",\"warm_2001_queries_ms\":"<<warm<<"}";
  }
  std::cout<<"],\"checks\":"<<checks<<",\"max_error\":"<<max_error<<",\"passed\":true}\n";
}
