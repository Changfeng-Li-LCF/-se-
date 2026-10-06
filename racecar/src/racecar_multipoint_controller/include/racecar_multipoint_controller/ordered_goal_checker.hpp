#pragma once
#include "nav2_core/goal_checker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>

namespace racecar_multipoint_controller {
// The controller supplies its ordered robot progress, never its ahead-of-car carrot.
class OrderedGoalChecker : public nav2_core::GoalChecker {
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    auto node=parent.lock();
    if (!node) throw std::runtime_error("Goal checker node expired");
    rcl_interfaces::msg::ParameterDescriptor desc;desc.read_only=true;
    auto declare=[&](const std::string &key,auto value) {
      if (!node->has_parameter(name+"."+key)) node->declare_parameter(name+"."+key,value,desc);
    };
    declare("xy_goal_tolerance",0.2);declare("yaw_goal_tolerance",1.5);declare("stateful",true);
    node->get_parameter(name+".xy_goal_tolerance",xy_tolerance_);
    node->get_parameter(name+".yaw_goal_tolerance",yaw_tolerance_);
    node->get_parameter(name+".stateful",stateful_);
    if (!std::isfinite(xy_tolerance_) || xy_tolerance_<=0 ||
        !std::isfinite(yaw_tolerance_) || yaw_tolerance_<0 || yaw_tolerance_>M_PI)
      throw std::invalid_argument("Invalid ordered goal tolerance");
    reset();
  }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_=false;check_xy_=true;progress_=length_=0;
  }
  void updateProgress(double progress,double length) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(progress) || !std::isfinite(length) || length<=0 ||
        progress<0 || progress>length+1e-6) {
      ready_=false;check_xy_=true;return;
    }
    // The controller resets this object on every replacement plan.
    progress_=progress;length_=length;ready_=true;
  }
  bool isGoalReached(const geometry_msgs::msg::Pose &query,
                     const geometry_msgs::msg::Pose &goal,
                     const geometry_msgs::msg::Twist &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    // Do not latch the XY test when the early branch passes near the endpoint.
    if (!ready_ || length_-progress_>xy_tolerance_+1e-9) {
      check_xy_=true;return false;
    }
    const double dx=query.position.x-goal.position.x,dy=query.position.y-goal.position.y;
    const double yaw=tf2::getYaw(query.orientation)-tf2::getYaw(goal.orientation);
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(yaw)) return false;
    if (check_xy_) {
      if (dx*dx+dy*dy>xy_tolerance_*xy_tolerance_) return false;
      if (stateful_) check_xy_=false;
    }
    return std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))<=yaw_tolerance_;
  }
  bool getTolerances(geometry_msgs::msg::Pose &pose,geometry_msgs::msg::Twist &velocity) override {
    const double unused=std::numeric_limits<double>::lowest();
    pose.position.x=pose.position.y=xy_tolerance_;pose.position.z=unused;
    pose.orientation.x=pose.orientation.y=0;
    pose.orientation.z=std::sin(yaw_tolerance_/2);pose.orientation.w=std::cos(yaw_tolerance_/2);
    velocity.linear.x=velocity.linear.y=velocity.linear.z=unused;
    velocity.angular.x=velocity.angular.y=velocity.angular.z=unused;
    return true;
  }
private:
  std::mutex mutex_;
  double xy_tolerance_=0.2,yaw_tolerance_=1.5,progress_=0,length_=0;
  bool stateful_=true,check_xy_=true,ready_=false;
};
} // namespace racecar_multipoint_controller
