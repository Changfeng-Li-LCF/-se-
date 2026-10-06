#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include "racecar_multipoint_controller/ordered_goal_checker.hpp"
#include "racecar_multipoint_controller/indexed_path.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace racecar_multipoint_controller;
void require(bool ok,const char *message) {if (!ok) throw std::runtime_error(message);}
geometry_msgs::msg::Pose pose(double x,double y,double yaw=0) {
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;
  p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);return p;
}
int main(int argc,char **argv) {
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core","nav2_core::Controller");
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
  auto controller=controllers.createSharedInstance("racecar_multipoint_controller::MultiPointController");
  auto plugin=goals.createSharedInstance("racecar_multipoint_controller::OrderedGoalChecker");
  auto checker=std::dynamic_pointer_cast<OrderedGoalChecker>(plugin);
  require(bool(controller) && bool(checker),"Plugin discovery/type cast failed");
  geometry_msgs::msg::Twist velocity;
  auto goal=pose(1,2);
  checker->reset();require(!checker->isGoalReached(goal,goal,velocity),"Uninitialized progress accepted");
  checker->updateProgress(1.6,18.22);
  require(!checker->isGoalReached(goal,goal,velocity),"Early endpoint crossing accepted");
  checker->updateProgress(18.1,18.22);
  require(!checker->isGoalReached(pose(3,2),goal,velocity),"Far from endpoint accepted");
  require(!checker->isGoalReached(pose(1,2,3),goal,velocity),"Incorrect heading accepted");
  require(checker->isGoalReached(goal,goal,velocity),"Actual completion rejected");
  checker->reset();require(!checker->isGoalReached(goal,goal,velocity),"Completion leaked into next goal");
  checker->updateProgress(NAN,18.22);require(!checker->isGoalReached(goal,goal,velocity),"Invalid progress accepted");
  if (argc==3) {
    std::ifstream pathfile(argv[1]),posefile(argv[2]);require(bool(pathfile)&&bool(posefile),"Missing replay data");
    std::vector<Point> points;Point point;
    while (pathfile>>point.x>>point.y) points.push_back(point);
    IndexedPath indexed;indexed.reset(points,.01);
    goal=pose(points.back().x,points.back().y,.1634216349941008);
    checker->reset();double x,y,yaw;unsigned close_passes=0,samples=0;
    while (posefile>>x>>y>>yaw) {
      indexed.update({x,y},.5);checker->updateProgress(indexed.progress,indexed.distances.back());
      if (std::hypot(x-goal.position.x,y-goal.position.y)<=.2) ++close_passes;
      require(!checker->isGoalReached(pose(x,y,yaw),goal,velocity),"Recorded premature stop still accepted");
      ++samples;
    }
    require(close_passes>0 && samples>20,"Replay did not cover reported failure");
    indexed.reset(points,.01);checker->reset();bool reached=false;
    for (auto p:points) {
      indexed.update(p,.5);checker->updateProgress(indexed.progress,indexed.distances.back());
      reached=checker->isGoalReached(pose(p.x,p.y,.1634216349941008),goal,velocity);
      if (indexed.distances.back()-indexed.progress>.2+1e-9)
        require(!reached,"Simulation cut across an earlier branch");
    }
    require(reached,"Full ordered figure-eight cannot finish");
    std::cout<<"PASS recorded failure replay: "<<samples<<" poses, "<<close_passes
             <<" early near-endpoint poses rejected; full figure-eight completes\n";
  }
  std::cout<<"PASS both plugins loaded, ordered gate, XY/yaw tolerances, reset, invalid input; no nodes or vehicle IO\n";
}
