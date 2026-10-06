#pragma once

#include <string>
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace racecar_multipoint_controller {
struct ReferencePose {
  geometry_msgs::msg::PoseStamped pose;
  double correction_lag_s=0.0;
  double pose_age_s=0.0;
  double correction_age_s=0.0;
};

// The input is the current base pose in the local costmap frame (odom).
// Hold only the map/odom correction across a short publishing skew, never the
// robot pose. Both preview selection and path inversion consume this one result.
inline ReferencePose transform_reference_pose(
    tf2_ros::Buffer &buffer, const geometry_msgs::msg::PoseStamped &pose,
    const std::string &reference_frame, tf2::Duration tolerance, tf2::TimePoint now) {
  const tf2::TimePoint requested(tf2::Duration(rclcpp::Time(pose.header.stamp).nanoseconds()));
  const auto pose_age=now-requested;
  if (pose.header.frame_id==reference_frame) return {pose,0.0,tf2::durationToSec(pose_age),0.0};
  geometry_msgs::msg::TransformStamped correction;
  double lag_s=0.0;
  try {
    // No blocking wait while the controller holds the costmap mutex.
    correction=buffer.lookupTransform(reference_frame,pose.header.frame_id,requested);
  } catch (const tf2::ExtrapolationException &) {
    correction=buffer.lookupTransform(reference_frame,pose.header.frame_id,tf2::TimePointZero);
    const tf2::TimePoint latest(tf2::Duration(rclcpp::Time(correction.header.stamp).nanoseconds()));
    const auto lag=requested-latest;
    // Check the two independent sources of age separately. Holding the map/odom
    // correction never freezes the current odometry pose. The total correction
    // age remains diagnostic rather than charging both ages against one limit.
    if (lag<tf2::Duration::zero() || lag>tolerance || pose_age>tolerance) {
      throw tf2::ExtrapolationException("Reference TF outside existing transform_tolerance: pose gap="+
        std::to_string(tf2::durationToSec(lag))+" s, pose age="+
        std::to_string(tf2::durationToSec(pose_age))+" s, correction age="+
        std::to_string(tf2::durationToSec(now-latest))+" s");
    }
    lag_s=tf2::durationToSec(lag);
  }
  ReferencePose result;
  tf2::doTransform(pose,result.pose,correction);
  // doTransform stamps the correction time; the robot pose still belongs to
  // the input odometry time, including when the correction is held briefly.
  result.pose.header.stamp=pose.header.stamp;
  result.correction_lag_s=lag_s;
  result.pose_age_s=tf2::durationToSec(pose_age);
  const tf2::TimePoint correction_stamp(
    tf2::Duration(rclcpp::Time(correction.header.stamp).nanoseconds()));
  result.correction_age_s=tf2::durationToSec(now-correction_stamp);
  return result;
}
}  // namespace racecar_multipoint_controller
