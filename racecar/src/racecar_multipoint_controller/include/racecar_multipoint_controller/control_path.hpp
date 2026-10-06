#pragma once
#include "racecar_multipoint_controller/indexed_path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace racecar_multipoint_controller {
// Regulation needs every position and the terminal heading. Intermediate
// headings/headers are display data: refresh them only when publishing the path.
// Keep the complete path and exact transform; reuse message storage each cycle.
inline void prepare_control_path(const IndexedPath &path, std::size_t first,
    Point robot, double robot_yaw, double terminal_yaw,
    const std_msgs::msg::Header &body_header, bool display, nav_msgs::msg::Path &out) {
  out.header=body_header;
  out.poses.resize(path.points.size()-first);
  const double yaw=-robot_yaw,c=std::cos(yaw),s=std::sin(yaw);
  for (std::size_t i=first;i<path.points.size();++i) {
    const auto p=path.points[i];
    auto &q=out.poses[i-first];
    const double dx=p.x-robot.x,dy=p.y-robot.y;
    q.pose.position.x=c*dx-s*dy;
    q.pose.position.y=s*dx+c*dy;
    if (display || i+1==path.points.size()) {
      q.header=body_header;
      const double heading=i+1<path.points.size() ?
        std::atan2(path.points[i+1].y-p.y,path.points[i+1].x-p.x) : terminal_yaw;
      q.pose.orientation.z=std::sin((heading+yaw)/2);
      q.pose.orientation.w=std::cos((heading+yaw)/2);
    }
  }
}
} // namespace racecar_multipoint_controller
