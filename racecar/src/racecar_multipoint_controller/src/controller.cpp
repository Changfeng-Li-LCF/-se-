// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Modifications: ordered single-point preview for the racecar, 2026.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Based on Navigation2 Humble RegulatedPurePursuitController command flow.

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "racecar_multipoint_controller/ordered_single_preview.hpp"
#include "racecar_multipoint_controller/control_path.hpp"
#include "racecar_multipoint_controller/ordered_goal_checker.hpp"
#include "tf2/utils.h"
#include "racecar_multipoint_controller/reference_transform.hpp"
#include "racecar_multipoint_controller/preview_handoff.hpp"
#include "racecar_multipoint_controller/curvature_handoff.hpp"
#include "racecar_multipoint_controller/handoff_costmap.hpp"
#include "racecar_multipoint_controller/motion_continuity.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/parameter_client.hpp"
#include <atomic>
#include <chrono>
#include <utility>

namespace racecar_multipoint_controller {
using Base=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

class MultiPointController : public Base {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    Base::configure(parent,name,tf,costmap);
    auto node=parent.lock();
    if (!node) throw std::runtime_error("Controller node expired");
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only=true; // Change saved settings, then restart the controller.
    auto declare=[&](const std::string &key, auto value) {
      if (!node->has_parameter(name+"."+key)) node->declare_parameter(name+"."+key,value,desc);
    };
    declare("multipoint_enabled",true);
    declare("tracking_mode",std::string("ordered_single"));
    declare("preview_spacing_m",0.01);
    declare("curvature_sample_spacing_m",0.3);
    declare("curvature_min_speed",1.5);
    declare("progress_search_ahead_m",0.5);
    declare("replan_handoff_tau_s",0.1);
    declare("terminal_deceleration_mps2",0.6);
    declare("terminal_response_time_s",0.2);
    declare("terminal_stop_margin_m",0.15);

    node->get_parameter(name+".multipoint_enabled",enabled_);
    node->get_parameter(name+".preview_spacing_m",spacing_);
    node->get_parameter(name+".curvature_sample_spacing_m",curvature_spacing_);
    node->get_parameter(name+".curvature_min_speed",curvature_min_speed_);
    node->get_parameter(name+".progress_search_ahead_m",search_ahead_);
    node->get_parameter(name+".replan_handoff_tau_s",handoff_tau_);
    node->get_parameter(name+".terminal_deceleration_mps2",terminal_deceleration_);
    node->get_parameter(name+".terminal_response_time_s",terminal_response_time_);
    node->get_parameter(name+".terminal_stop_margin_m",terminal_stop_margin_);
    if (!std::isfinite(terminal_deceleration_) || terminal_deceleration_<=0 ||
        !std::isfinite(terminal_response_time_) || terminal_response_time_<0 ||
        !std::isfinite(terminal_stop_margin_) || terminal_stop_margin_<0)
      throw std::invalid_argument("Invalid terminal deceleration profile");
    RCLCPP_INFO(logger_,"terminal-deceleration-v1: enabled=%s a=%.3f m/s2 response=%.3f s margin=%.3f m; ordered remaining full-route length",
      approach_velocity_scaling_dist_>0.0 ? "true" : "false",terminal_deceleration_,terminal_response_time_,terminal_stop_margin_);
    if (!std::isfinite(handoff_tau_) || handoff_tau_<=0) throw std::invalid_argument("Invalid replan handoff time constant");
    RCLCPP_INFO(logger_,"replan-curve-handoff-v4: replacement-delta continuity tau=%.3f s; goal route takes precedence over optional motion preference",handoff_tau_);
    RCLCPP_INFO(logger_,"open-space-motion-v1: goal-priority-v1: unpassed goals bound geometric splices; old-turn release is bounded; no new stop condition");
    std::string mode;node->get_parameter(name+".tracking_mode",mode);
    if (!enabled_ || mode!="ordered_single") throw std::invalid_argument("Ordered single preview must remain enabled; legacy multipoint_enabled is the custom-plugin switch");
    if (enabled_) {
      if (!std::isfinite(lookahead_dist_) || lookahead_dist_<=0 || !std::isfinite(spacing_) || spacing_<=0 ||
          !std::isfinite(search_ahead_) || search_ahead_<=0) throw std::invalid_argument("Invalid indexed preview settings");
      if (allow_reversing_) throw std::invalid_argument("Ordered single preview requires allow_reversing: false");

    }
    if (!std::isfinite(curvature_min_speed_) || curvature_min_speed_<=0 ||
        curvature_min_speed_>desired_linear_vel_ || regulated_linear_scaling_min_speed_>curvature_min_speed_)
      throw std::invalid_argument("Invalid independent curvature/obstacle speed floors");
    RCLCPP_INFO(logger_,"independent-speed-floors-v1: cruise=%.2f curvature_floor=%.2f obstacle_floor=%.2f m/s",
      desired_linear_vel_,curvature_min_speed_,regulated_linear_scaling_min_speed_);
    if (!std::isfinite(curvature_spacing_) || curvature_spacing_<spacing_)
      throw std::invalid_argument("Invalid curvature chord spacing");
    if (!std::isfinite(lookahead_time_) || lookahead_time_<=0 ||
        !std::isfinite(min_lookahead_dist_) || min_lookahead_dist_<=0 ||
        !std::isfinite(max_lookahead_dist_) || max_lookahead_dist_<min_lookahead_dist_)
      throw std::invalid_argument("Invalid adaptive lookahead bounds/time");
    RCLCPP_INFO(logger_,"official-speed-lookahead-v1: adaptive=%s T=%.2f s bounds=[%.2f, %.2f] m; per control cycle",
      use_velocity_scaled_lookahead_dist_ ? "true" : "false",lookahead_time_,min_lookahead_dist_,max_lookahead_dist_);
    RCLCPP_INFO(logger_,"control-work-v1: reuse full-path storage; intermediate headings only at path publication; unchanged regulation/preview");
    RCLCPP_INFO(logger_,"tf-latency-v1: independent TF time gates; costmap lock limited to grid reads");
    samples_pub_=node->create_publisher<nav_msgs::msg::Path>(name+"/lookahead_samples",10);
    global_grid_cache_=std::make_shared<GlobalGridCache>();
    const auto cache=global_grid_cache_;
    global_grid_sub_=node->create_subscription<nav2_msgs::msg::Costmap>(
      "global_costmap/costmap_raw",rclcpp::QoS(1).reliable().transient_local(),
      [cache](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
        std::atomic_store(&cache->message,std::move(msg));
      });
    remaining_goals_cache_=std::make_shared<RemainingGoalsCache>();
    const auto goals_cache=remaining_goals_cache_;
    remaining_goals_sub_=node->create_subscription<nav_msgs::msg::Path>(
      "/navigation/remaining_goals",rclcpp::QoS(1).reliable().durability_volatile(),
      [goals_cache](nav_msgs::msg::Path::ConstSharedPtr msg) {
        auto snapshot=std::make_shared<RemainingGoalsSnapshot>();
        snapshot->message=std::move(msg);snapshot->received=std::chrono::steady_clock::now();
        std::atomic_store(&goals_cache->snapshot,std::move(snapshot));
      });
    const std::string ns=node->get_namespace();
    planner_parameters_=std::make_shared<rclcpp::AsyncParametersClient>(
      node,(ns=="/" ? std::string() : ns)+"/planner_server");
    refreshPlannerConstraints();
    RCLCPP_INFO(logger_,"Ordered single preview: radius %.3f m, IDs spaced %.3f m, progress search %.2f m",
                lookahead_dist_,spacing_,search_ahead_);
  }
  void activate() override {Base::activate();samples_pub_->on_activate();}
  void deactivate() override {
    samples_pub_->on_deactivate();Base::deactivate();
    std::lock_guard<std::mutex> lock(mutex_);
    clearHandoff();last_preview_valid_=cycle_initialized_=false;
  }
  void cleanup() override {
    remaining_goals_sub_.reset();remaining_goals_cache_.reset();
    global_grid_sub_.reset();global_grid_cache_.reset();planner_parameters_.reset();
    planner_future_={};planning_constraints_ready_=false;planning_radius_=0;
    planning_allow_unknown_=false;last_constraint_request_={};
    samples_pub_.reset();Base::cleanup();
  }

  void setPlan(const nav_msgs::msg::Path &path) override {
    std::vector<Point> input;input.reserve(path.poses.size());
    for (const auto &p:path.poses) input.push_back({p.pose.position.x,p.pose.position.y});
    IndexedPath replacement;replacement.reset(input,spacing_);
    std::lock_guard<std::mutex> lock(mutex_);
    const bool same_frame=full_path_.header.frame_id==path.header.frame_id;
    const bool continuing=same_frame && last_preview_valid_ && cycle_initialized_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-last_cycle_).count()<=0.5;
    const bool geometry_changed=!same_geometry(indexed_,replacement);
    if (!continuing || geometry_changed) {
      if (continuing) {
        // tracking_ remains the last actually executed geometry, even when
        // several plans arrive before the next controller cycle.
        handoff_pending_=true;
      } else {
        clearHandoff();last_preview_valid_=false;
        tracking_=replacement;last_target_progress_=0;
      }
      indexed_=std::move(replacement);
    }
    replan_changed_geometry_=replan_changed_geometry_ || geometry_changed;
    if (continuing && last_handoff_mapped_) handoff_pending_=true;
    full_path_=path;global_plan_=path;debug_cycle_=0;plan_changed_=true;
    ++goal_limit_plan_revision_;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *goal_checker) override {
    const auto started=std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock_reinit(mutex_);
    const double handoff_dt=cycle_initialized_ ?
      std::chrono::duration<double>(started-last_cycle_).count() : 0.0;
    // Discard an idle continuity cache, without changing any motion timeout.
    if (!cycle_initialized_ || handoff_dt>0.5) {
      clearHandoff();last_preview_valid_=false;tracking_=indexed_;last_target_progress_=0;
    }
    auto *costmap=costmap_ros_->getCostmap();
    // TF, preview calculations and debug publishing do not read costmap cells.
    // Lock only the actual grid reads below so these operations cannot delay
    // updates while holding the grid mutex, or age the pose before TF lookup.
    double costmap_wait_ms=0.0,tf_lookup_ms=0.0,pose_age_ms=0.0,correction_age_ms=0.0;
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (goal_checker && goal_checker->getTolerances(pose_tolerance,vel_tolerance)) {
      goal_dist_tol_=pose_tolerance.position.x;
    }
    if (indexed_.points.empty()) throw nav2_core::PlannerException("No indexed reference plan");
    geometry_msgs::msg::PoseStamped robot;
    const auto tf_started=std::chrono::steady_clock::now();
    try {
      const auto reference=transform_reference_pose(*tf_,pose,full_path_.header.frame_id,
        transform_tolerance_,tf2::TimePoint(tf2::Duration(clock_->now().nanoseconds())));
      robot=reference.pose;
      tf_lookup_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-tf_started).count();
      pose_age_ms=reference.pose_age_s*1000.0;
      correction_age_ms=reference.correction_age_s*1000.0;
      if (reference.correction_lag_s>0.0) {
        RCLCPP_WARN_THROTTLE(logger_,*clock_,1000,
          "Reference TF correction held %.3f ms; current odometry pose retained; pose_age_ms=%.3f correction_age_ms=%.3f tf_lookup_ms=%.3f",
          1000.0*reference.correction_lag_s,pose_age_ms,correction_age_ms,tf_lookup_ms);
      }
    } catch (const tf2::TransformException &error) {
      tf_lookup_ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-tf_started).count();
      throw nav2_core::PlannerException(std::string("Cannot transform robot pose to reference frame: ")+
        error.what()+"; tf_lookup_ms="+std::to_string(tf_lookup_ms));
    }
    auto *ordered_goal=dynamic_cast<OrderedGoalChecker *>(goal_checker);
    if (!ordered_goal) throw nav2_core::PlannerException("Ordered tracking requires OrderedGoalChecker");
    if (plan_changed_) { ordered_goal->reset();plan_changed_=false; }
    indexed_.update({robot.pose.position.x,robot.pose.position.y},search_ahead_);
    ordered_goal->updateProgress(indexed_.progress,indexed_.distances.back());
    if (!std::isfinite(speed.linear.x))
      throw nav2_core::PlannerException("Nonfinite speed for adaptive lookahead");
    const double lookahead=getLookAheadDistance(speed);
    if (!std::isfinite(lookahead) || lookahead<=0)
      throw nav2_core::PlannerException("Invalid adaptive lookahead distance");
    const Point robot_point{robot.pose.position.x,robot.pose.position.y};
    const double robot_yaw=tf2::getYaw(robot.pose.orientation);
    tracking_.update(robot_point,search_ahead_);
    std::optional<SinglePreview> raw_target;
    try {raw_target=ordered_single_preview(indexed_,robot_point,robot_yaw,lookahead,indexed_.progress);}
    catch (const std::invalid_argument &) { /* Keep valid executed geometry as fallback. */ }
    const auto handoff_started=std::chrono::steady_clock::now();
    goal_join_limit_=remainingGoalJoinLimit();
    refreshPlannerConstraints();
    prepareMotionSpace(robot,pose,lookahead,costmap_wait_ms);
    std::optional<double> replaced_curvature;
    if (handoff_pending_ && last_preview_valid_) {
      // Compare both references at THIS pose. Last cycle's curvature would
      // also suppress legitimate corrections caused by vehicle motion.
      try {
        replaced_curvature=ordered_single_preview(tracking_,robot_point,robot_yaw,lookahead,
          preview_search_floor(use_velocity_scaled_lookahead_dist_,tracking_.progress,last_target_progress_)).curvature;
      } catch (const std::invalid_argument &) {
        // An unusable old reference must not delay the new forward reference.
        curvature_handoff_.clear();
      }
    }
    if (handoff_pending_) {
      const auto transition=buildCurveHandoff(robot,pose,lookahead,costmap_wait_ms);
      if (transition) {
        tracking_=transition->path;last_target_progress_=transition->target_floor;
        last_handoff_mapped_=true;
        last_kept_prefix_m_=transition->kept_prefix_m;
        last_reference_error_m_=transition->reference_error_m;
        handoff_status_=transition->retained_prefix ? "short_prefix_convergence" : "pose_curve_convergence";
      } else {
        // An unsafe/incompatible old prefix never blocks a fresh planner path.
        tracking_=indexed_;last_target_progress_=tracking_.progress;last_handoff_mapped_=false;
        last_kept_prefix_m_=last_reference_error_m_=0;
      }
      handoff_pending_=false;
      replan_changed_geometry_=false;
    }
    if (raw_target && last_handoff_mapped_) {
      try {
        const auto accepted=ordered_single_preview(tracking_,robot_point,robot_yaw,lookahead,
          preview_search_floor(use_velocity_scaled_lookahead_dist_,tracking_.progress,last_target_progress_));
        const bool opposing=accepted.curvature*raw_target->curvature<0 &&
          std::abs(accepted.curvature-raw_target->curvature)>4*spacing_/(lookahead*lookahead);
        if (opposing || distance(accepted.point,raw_target->point)>0.10*lookahead ||
            goal_join_limit_-indexed_.progress<1.3*lookahead) {
          tracking_=indexed_;last_target_progress_=indexed_.progress;last_handoff_mapped_=false;
          handoff_status_="goal_route_priority";curvature_handoff_.clear();
          last_kept_prefix_m_=last_reference_error_m_=0;
        }
      } catch (const std::invalid_argument &) {
        tracking_=indexed_;last_target_progress_=indexed_.progress;last_handoff_mapped_=false;
        curvature_handoff_.clear();
      }
    }
    const double handoff_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-handoff_started).count();
    const auto path_started=std::chrono::steady_clock::now();
    const auto first=tracking_.next_index();
    const bool publish_plan=(debug_cycle_++ % 5)==0;
    auto body_header=robot.header;body_header.frame_id=costmap_ros_->getBaseFrameID();
    auto &plan=control_plan_;
    prepare_control_path(tracking_,first,robot_point,robot_yaw,
      tf2::getYaw(full_path_.poses.back().pose.orientation),body_header,publish_plan,plan);
    const double path_prepare_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-path_started).count();
    auto select_target=[&]() {
      try {
        return ordered_single_preview(tracking_,robot_point,robot_yaw,lookahead,
          preview_search_floor(use_velocity_scaled_lookahead_dist_,tracking_.progress,last_target_progress_));
      } catch (const std::invalid_argument &error) {
        throw nav2_core::PlannerException(error.what());
      }
    };
    auto target=select_target();
    if (replaced_curvature) curvature_handoff_.join(*replaced_curvature,target.curvature);
    // Only the replacement delta decays. Pose-driven changes on an unchanged
    // path pass through immediately; frequent replacements do not reset time.
    const double requested_curvature=curvature_handoff_.update(target.curvature,handoff_dt,handoff_tau_);
    const double raw_curvature=raw_target ? raw_target->curvature : target.curvature;
    const double handoff_shift=raw_target ? distance(target.point,raw_target->point) : 0;
    const double command_curvature=applyMotionContinuity(requested_curvature,raw_curvature,
      handoff_dt,lookahead,raw_target.has_value());
    if (last_handoff_mapped_) target.selection="curve_handoff";
    last_target_progress_=use_velocity_scaled_lookahead_dist_ ? target.progress :
      std::max(last_target_progress_,target.progress);
    const auto debug_started=std::chrono::steady_clock::now();
    if (publish_plan) global_path_pub_->publish(plan);

    nav_msgs::msg::Path samples;samples.header=plan.header;
    geometry_msgs::msg::PoseStamped sample;sample.header=plan.header;
    sample.pose.position.x=target.point.x;sample.pose.position.y=target.point.y;
    sample.pose.orientation.w=1.0;samples.poses.push_back(sample);
    samples_pub_->publish(samples);
    const auto &carrot=samples.poses.back();
    carrot_pub_->publish(createCarrotMsg(carrot));

    const double debug_publish_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-debug_started).count();
    const double route_curvature=plannedCurvature(lookahead);
    double linear=desired_linear_vel_,angular=0.0,sign=1.0,angle=0.0;
    if (shouldRotateToGoalHeading(carrot)) {
      rotateToHeading(linear,angular,tf2::getYaw(plan.poses.back().pose.orientation),speed);
    } else if (shouldRotateToPath(carrot,angle)) {
      rotateToHeading(linear,angular,angle,speed);
    } else {
      // The original full/pruned plan remains intact for endpoint handling.
      {
        const auto grid_wait_started=std::chrono::steady_clock::now();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        costmap_wait_ms+=std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-grid_wait_started).count();
        applyIndependentSpeedConstraints(std::max({std::abs(command_curvature),std::abs(target.curvature),std::abs(raw_curvature),route_curvature}),speed,costAtPose(pose.pose.position.x,pose.pose.position.y),
                         plan,linear,sign);
      }
      // Limit only by the remaining ordered route, never distance to an
      // intermediate waypoint or the end of a cropped local-costmap plan.
      // Share Nav2's approach setting for both the enable switch and distance.
      const double remaining=std::max(0.0,indexed_.distances.back()-indexed_.progress);
      if (approach_velocity_scaling_dist_>0.0 && remaining<approach_velocity_scaling_dist_) {
        const double response_speed=terminal_deceleration_*terminal_response_time_;
        const double terminal_cap=std::max(min_approach_linear_velocity_,
          std::sqrt(response_speed*response_speed+2*terminal_deceleration_*
            std::max(0.0,remaining-terminal_stop_margin_))-response_speed);
        linear=std::copysign(std::min(std::abs(linear),terminal_cap),linear);
      }
      angular=linear*command_curvature;
    }
    const double horizon=std::hypot(carrot.pose.position.x,carrot.pose.position.y);
    // Check the actual commanded arc, including a replacement transition.
    if (use_collision_detection_) {
      const auto grid_wait_started=std::chrono::steady_clock::now();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
      costmap_wait_ms+=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-grid_wait_started).count();
      if (isCollisionImminent(pose,linear,angular,horizon)) {
        clearHandoff();last_preview_valid_=false;
        throw nav2_core::PlannerException("Ordered single controller detected collision ahead");
      }
    }
    geometry_msgs::msg::TwistStamped cmd;cmd.header=pose.header;
    cmd.twist.linear.x=linear;cmd.twist.angular.z=angular;
    if (std::abs(linear)<1e-9) clearHandoff();
    last_cycle_=started;cycle_initialized_=true;last_preview_valid_=std::abs(linear)>=1e-9;
    last_executed_curvature_=std::abs(linear)>=1e-9 ? angular/linear : 0;
    last_command_v_=linear;
    const double ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-started).count();
    const double command_pose_age_ms=1000.0*(clock_->now()-rclcpp::Time(pose.header.stamp)).seconds();
    RCLCPP_INFO_THROTTLE(logger_,*clock_,1000,
      "single lookahead=%.3f speed=%.3f progress=%.3f remaining=%.3f next_id=%zu target_segment=%zu fraction=%.3f target_s=%.3f distance=%.3f selection=%s curvature=%.3f command_curvature=%.3f handoff_curvature_offset=%.3f raw_curvature=%.3f handoff_shift_m=%.3f handoff_mapped=%s route_curvature=%.3f command_v=%.3f compute_ms=%.3f tf_lookup_ms=%.3f costmap_wait_ms=%.3f pose_age_ms=%.3f command_pose_age_ms=%.3f correction_age_ms=%.3f path_prepare_ms=%.3f handoff_ms=%.3f debug_publish_ms=%.3f",
      lookahead,speed.linear.x,indexed_.progress,indexed_.distances.back()-indexed_.progress,first,target.segment,target.fraction,target.progress,horizon,target.selection,target.curvature,last_executed_curvature_,curvature_handoff_.offset(),raw_curvature,handoff_shift,last_handoff_mapped_ ? "true" : "false",route_curvature,linear,ms,tf_lookup_ms,costmap_wait_ms,pose_age_ms,command_pose_age_ms,correction_age_ms,path_prepare_ms,handoff_ms,debug_publish_ms);
    if (ms>control_duration_*1000.0) {
      RCLCPP_WARN_THROTTLE(logger_,*clock_,5000,"Ordered single controller exceeded cycle budget: %.2f ms",ms);
    }
    RCLCPP_INFO_THROTTLE(logger_,*clock_,1000,
      "terminal_deceleration remaining=%.3f stopping_distance=%.3f command_v=%.3f enabled=%s",
      indexed_.distances.back()-indexed_.progress,
      std::abs(speed.linear.x)*terminal_response_time_+speed.linear.x*speed.linear.x/(2*terminal_deceleration_)+terminal_stop_margin_,linear,
      approach_velocity_scaling_dist_>0.0 ? "true" : "false");
    RCLCPP_INFO_THROTTLE(logger_,*clock_,1000,
      "curve_handoff status=%s tracking_progress=%.3f full_route_progress=%.3f planner_radius=%.3f allow_unknown=%s kept_prefix_m=%.3f reference_error_m=%.3f goal_join_remaining_m=%.3f goal_priority_active=%s",
      handoff_status_.c_str(),tracking_.progress,indexed_.progress,planning_radius_,planning_allow_unknown_ ? "true" : "false",
      last_kept_prefix_m_,last_reference_error_m_,goal_join_limit_-indexed_.progress,
      goal_priority_blend_.active() ? "true" : "false");
    RCLCPP_INFO_THROTTLE(logger_,*clock_,1000,
      "motion_continuity openness=%.3f requested_curvature=%.3f applied_curvature=%.3f",
      motion_preference_,requested_curvature,last_executed_curvature_);
    return cmd;
  }
private:
  struct GlobalGridCache {nav2_msgs::msg::Costmap::ConstSharedPtr message;};
  struct RemainingGoalsSnapshot {
    nav_msgs::msg::Path::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received;
  };
  struct RemainingGoalsCache {std::shared_ptr<RemainingGoalsSnapshot> snapshot;};
  double remainingGoalJoinLimit() {
    if (!remaining_goals_cache_) return indexed_.progress;
    const auto snapshot=std::atomic_load(&remaining_goals_cache_->snapshot);
    if (!snapshot || std::chrono::duration<double>(
        std::chrono::steady_clock::now()-snapshot->received).count()>0.5) return indexed_.progress;
    const auto &goals=*snapshot->message;
    if (goals.header.frame_id!=full_path_.header.frame_id || goals.poses.empty()) return indexed_.progress;
    // A newly committed path may precede its auxiliary message by one callback.
    // Follow that raw path until the matching goals arrive; never wait or stop.
    if (goals.header.stamp!=full_path_.header.stamp) return indexed_.progress;
    if (goal_limit_snapshot_==snapshot && goal_limit_cache_revision_==goal_limit_plan_revision_)
      return goal_limit_cached_;
    goal_limit_snapshot_=snapshot;goal_limit_cache_revision_=goal_limit_plan_revision_;
    const auto &p=goals.poses.front().pose.position;
    const auto projection=project_local(indexed_,{p.x,p.y},indexed_.progress,indexed_.distances.back());
    // The planner may approach a goal with its existing position tolerance.
    // An unmatched point disables only the optional splice, not navigation.
    if (!projection.valid) return goal_limit_cached_=indexed_.progress;
    // Use the FIRST encounter with the goal region, not the globally nearest
    // later pass through the same area of an 8-shaped route. The untouched
    // suffix still contains the planner's actual waypoint approach.
    const double match_radius=std::max(goal_dist_tol_,projection.error)+2*spacing_;
    const auto first=indexed_.next_index();
    for (std::size_t j=first;j<indexed_.points.size();++j) {
      const auto local=project_local(indexed_,{p.x,p.y},
        std::max(indexed_.progress,indexed_.distances[j-1]),indexed_.distances[j]);
      if (local.valid && local.error<=match_radius) return goal_limit_cached_=local.progress;
    }
    return goal_limit_cached_=indexed_.progress;
  }
  void clearHandoff() {
    curvature_handoff_.clear();goal_priority_blend_.clear();
    handoff_pending_=false;last_handoff_mapped_=false;handoff_status_="raw_plan";
    last_kept_prefix_m_=last_reference_error_m_=0;
    motion_preference_=0;
  }

  void prepareMotionSpace(const geometry_msgs::msg::PoseStamped &robot,
      const geometry_msgs::msg::PoseStamped &pose,double lookahead,double &costmap_wait_ms) {
    motion_preference_=0;
    // Goal accuracy and the first command retain the existing tracking behavior.
    if (!last_preview_valid_ || !planning_constraints_ready_ || !global_grid_cache_ ||
        std::min(indexed_.distances.back(),goal_join_limit_)-indexed_.progress<=1.3*lookahead ||
        pose.header.frame_id!=costmap_ros_->getGlobalFrameID()) return;
    motion_global_=std::atomic_load(&global_grid_cache_->message);
    if (!motion_global_ || motion_global_->header.frame_id!=full_path_.header.frame_id ||
        motion_global_->metadata.resolution<=0 || !motion_global_->metadata.size_x ||
        !motion_global_->metadata.size_y || motion_global_->data.size()!=
          static_cast<std::size_t>(motion_global_->metadata.size_x)*motion_global_->metadata.size_y ||
        std::abs(motion_global_->metadata.origin.orientation.z)>1e-6) return;
    const double age=(clock_->now()-rclcpp::Time(motion_global_->header.stamp)).seconds();
    if (age<0 || age>0.5) return;
    motion_global_grid_={motion_global_->data.data(),motion_global_->metadata.size_x,
      motion_global_->metadata.size_y,motion_global_->metadata.resolution,
      motion_global_->metadata.origin.position.x,motion_global_->metadata.origin.position.y};
    motion_footprint_.clear();double body_radius=0;
    for (const auto &p:costmap_ros_->getUnpaddedRobotFootprint()) {
      motion_footprint_.push_back({p.x,p.y});body_radius=std::max(body_radius,std::hypot(p.x,p.y));
    }
    if (motion_footprint_.size()<3) return;
    motion_robot_={{robot.pose.position.x,robot.pose.position.y},tf2::getYaw(robot.pose.orientation),0};
    motion_local_robot_={{pose.pose.position.x,pose.pose.position.y},tf2::getYaw(pose.pose.orientation),0};
    double raw_curvature=0;
    try {
      raw_curvature=ordered_single_preview(indexed_,motion_robot_.point,
        motion_robot_.yaw,lookahead,indexed_.progress).curvature;
    } catch (const std::invalid_argument &) {return;}
    auto *local=costmap_ros_->getCostmap();
    const auto wait_started=std::chrono::steady_clock::now();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(local->getMutex()));
      costmap_wait_ms+=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-wait_started).count();
      motion_local_grid_.width=local->getSizeInCellsX();motion_local_grid_.height=local->getSizeInCellsY();
      motion_local_grid_.resolution=local->getResolution();motion_local_grid_.origin_x=local->getOriginX();
      motion_local_grid_.origin_y=local->getOriginY();
      const auto count=static_cast<std::size_t>(motion_local_grid_.width)*motion_local_grid_.height;
      motion_local_data_.assign(local->getCharMap(),local->getCharMap()+count);
    }
    motion_local_grid_.data=motion_local_data_.data();
    double openness=1;
    // Check both the previous motion and the requested arc in the same pose.
    // Unknown cells never count as spacious, even if exploration allows them.
    for (const double k:{last_executed_curvature_,raw_curvature}) {
      const auto global_arc=motion_arc(motion_robot_.point,motion_robot_.yaw,k,lookahead);
      const auto local_arc=motion_arc(motion_local_robot_.point,motion_local_robot_.yaw,k,lookahead);
      openness=std::min({openness,motion_space_weight(motion_global_grid_,global_arc,body_radius,lookahead),
        motion_space_weight(motion_local_grid_,local_arc,body_radius,lookahead)});
      if (openness<=0) break;
    }
    motion_preference_=openness;
  }

  double applyMotionContinuity(double requested,double reference,double dt,double lookahead,
      bool reference_valid) {
    auto goal_priority=[&](double candidate) {
      return reference_valid ? goal_priority_blend_.update(candidate,reference,last_executed_curvature_,
        dt,handoff_tau_,0.2/lookahead,4*spacing_/(lookahead*lookahead)) : candidate;
    };
    auto original=[&]() {
      if (goal_priority_blend_.active()) {goal_priority_blend_.clear();return reference;}
      return requested;
    };
    if (motion_preference_<=0) {
      const auto candidate=goal_priority(requested);
      (void)candidate;return original();
    }
    double candidate=continuous_motion_curvature(last_executed_curvature_,requested,
      dt,lookahead,last_command_v_,motion_preference_,handoff_tau_);
    candidate=goal_priority(candidate);
    if (std::abs(candidate)>1/planning_radius_+1e-6) {
      motion_preference_=0;return original();
    }
    auto global_arc=motion_arc(motion_robot_.point,motion_robot_.yaw,candidate,lookahead);
    auto local_arc=motion_arc(motion_local_robot_.point,motion_local_robot_.yaw,candidate,lookahead);
    double body_radius=0;
    for (const auto &p:motion_footprint_) body_radius=std::max(body_radius,std::hypot(p.x,p.y));
    const double candidate_space=std::min(
      motion_space_weight(motion_global_grid_,global_arc,body_radius,lookahead),
      motion_space_weight(motion_local_grid_,local_arc,body_radius,lookahead));
    if (candidate_space<=0) {motion_preference_=0;return original();}
    motion_preference_=std::min(motion_preference_,candidate_space);
    for (std::size_t i=0;i<global_arc.size();++i) {
      if (!handoff_footprint_cost(motion_global_grid_,global_arc[i],motion_footprint_,false,true) ||
          !handoff_footprint_cost(motion_local_grid_,local_arc[i],motion_footprint_,false,true)) {
        // Reject ONLY the preference. The existing controller/obstacle policy
        // receives its original command; no protection latch or stop is added.
        motion_preference_=0;return original();
      }
    }
    return candidate;
  }

  // Read the planner's actual shared constraints without waiting for services
  // or introducing another configurable copy of its radius/unknown policy.
  void refreshPlannerConstraints() {
    if (!planner_parameters_) return;
    if (planner_future_.valid()) {
      if (planner_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return;
      try {
        const auto parameters=planner_future_.get();
        if (parameters.size()==2 && parameters[0].get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameters[1].get_type()==rclcpp::ParameterType::PARAMETER_BOOL &&
            std::isfinite(parameters[0].as_double()) && parameters[0].as_double()>0) {
          planning_radius_=parameters[0].as_double();planning_allow_unknown_=parameters[1].as_bool();
          planning_constraints_ready_=true;
        }
      } catch (const std::exception &) { /* Keep last confirmed constraints. */ }
      planner_future_={};
    }
    const auto now=std::chrono::steady_clock::now();
    if ((!planning_constraints_ready_ || std::chrono::duration<double>(now-last_constraint_request_).count()>=1.0) &&
        planner_parameters_->service_is_ready()) {
      last_constraint_request_=now;
      try {planner_future_=planner_parameters_->get_parameters({
        "GridBased.minimum_turning_radius","GridBased.allow_unknown"});}
      catch (const std::exception &) { /* The raw planner path remains usable. */ }
    }
  }

  std::optional<CurveHandoff> buildCurveHandoff(const geometry_msgs::msg::PoseStamped &robot,
      const geometry_msgs::msg::PoseStamped &pose,double lookahead,double &costmap_wait_ms) {
    if (goal_join_limit_-indexed_.progress<1.3*lookahead) {
      handoff_status_="unpassed_goal_uses_raw_route";return std::nullopt;
    }
    handoff_status_="planner_constraints_pending";
    if (!planning_constraints_ready_ || !global_grid_cache_) return std::nullopt;
    const auto global=std::atomic_load(&global_grid_cache_->message);
    handoff_status_="global_grid_pending";
    if (!global || global->header.frame_id!=full_path_.header.frame_id ||
        global->metadata.resolution<=0 || !global->metadata.size_x || !global->metadata.size_y ||
        global->data.size()!=static_cast<std::size_t>(global->metadata.size_x)*global->metadata.size_y ||
        std::abs(global->metadata.origin.orientation.z)>1e-6) return std::nullopt;
    const double age=(clock_->now()-rclcpp::Time(global->header.stamp)).seconds();
    if (age<0 || age>0.5 || pose.header.frame_id!=costmap_ros_->getGlobalFrameID()) return std::nullopt;
    HandoffGrid global_grid{global->data.data(),global->metadata.size_x,global->metadata.size_y,
      global->metadata.resolution,global->metadata.origin.position.x,global->metadata.origin.position.y};
    std::vector<unsigned char> local_data;HandoffGrid local_grid;
    std::vector<Point> footprint;
    for (const auto &p:costmap_ros_->getUnpaddedRobotFootprint()) footprint.push_back({p.x,p.y});
    auto *local=costmap_ros_->getCostmap();
    const auto wait_started=std::chrono::steady_clock::now();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(local->getMutex()));
      costmap_wait_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wait_started).count();
      local_grid.width=local->getSizeInCellsX();local_grid.height=local->getSizeInCellsY();
      local_grid.resolution=local->getResolution();local_grid.origin_x=local->getOriginX();local_grid.origin_y=local->getOriginY();
      const auto count=static_cast<std::size_t>(local_grid.width)*local_grid.height;
      local_data.assign(local->getCharMap(),local->getCharMap()+count);
    }
    local_grid.data=local_data.data();
    const Point robot_point{robot.pose.position.x,robot.pose.position.y},local_robot{pose.pose.position.x,pose.pose.position.y};
    const double yaw=tf2::getYaw(robot.pose.orientation),local_yaw=tf2::getYaw(pose.pose.orientation);
    auto validate=[&](const std::vector<CurveState> &curve,bool full_body)->std::optional<double> {
      double total=0;
      for (const auto &state:curve) {
        if (!full_body) {
          const int x=static_cast<int>(std::floor((state.point.x-global_grid.origin_x)/global_grid.resolution));
          const int y=static_cast<int>(std::floor((state.point.y-global_grid.origin_y)/global_grid.resolution));
          if (x<0 || y<0 || x>=static_cast<int>(global_grid.width) || y>=static_cast<int>(global_grid.height)) return std::nullopt;
          const auto cost=global_grid.data[static_cast<std::size_t>(y)*global_grid.width+x];
          if (cost==254 || (cost==255 && !planning_allow_unknown_)) return std::nullopt;
          total+=cost==255 ? 1.0 : static_cast<double>(cost)/253;
        } else {
          const auto global_cost=handoff_footprint_cost(global_grid,state,footprint,planning_allow_unknown_,true);
          if (!global_cost) return std::nullopt;
          // Both poses describe the same source time. Convert through that
          // common body frame instead of looking up a second, newer map TF.
          const CurveState local_state{
            preview_world(preview_body(state.point,robot_point,yaw),local_robot,local_yaw),
            state.yaw-yaw+local_yaw,state.curvature};
          const auto local_cost=handoff_footprint_cost(local_grid,local_state,footprint,planning_allow_unknown_,false);
          if (!local_cost) return std::nullopt;
          total+=std::max(*global_cost,*local_cost);
        }
      }
      return total/curve.size();
    };
    if (!replan_changed_geometry_ && last_handoff_mapped_) {
      std::vector<CurveState> near;
      const double end=std::min(tracking_.distances.back(),tracking_.progress+2*lookahead);
      for (double s=tracking_.progress;s<end;s+=0.02) near.push_back(path_state(tracking_,s));
      near.push_back(path_state(tracking_,end));
      const bool radius_ok=std::all_of(near.begin(),near.end(),[&](const auto &p) {
        return std::abs(p.curvature)<=1/planning_radius_+1e-6;
      });
      const auto reference=handoff_reference(tracking_,indexed_,robot_point,yaw,lookahead,
        preview_search_floor(use_velocity_scaled_lookahead_dist_,tracking_.progress,last_target_progress_),spacing_);
      // Even an unchanged raw plan must pull a displaced accepted curve back
      // toward itself. Reuse is only permitted after convergence, not merely
      // because the old curve is collision-free.
      const bool converged=reference && !reference->opposing && reference->separation<=2*spacing_ &&
        std::abs(reference->old_target.curvature-reference->new_target.curvature)<=4*spacing_/(lookahead*lookahead);
      if (radius_ok && converged && validate(near,true)) {
        CurveHandoff retained;retained.path=tracking_;retained.target_floor=last_target_progress_;
        retained.kept_prefix_m=0;retained.reference_error_m=reference->separation;
        retained.retained_prefix=true;return retained;
      }
      if (!radius_ok) {handoff_status_="retained_curve_invalid";return std::nullopt;}
    }
    handoff_status_="curve_infeasible_or_incompatible";
    return make_curve_handoff(tracking_,indexed_,robot_point,yaw,lookahead,
      preview_search_floor(use_velocity_scaled_lookahead_dist_,tracking_.progress,last_target_progress_),
      last_executed_curvature_,planning_radius_,spacing_,search_ahead_,validate,motion_preference_,goal_join_limit_);
  }
  void applyIndependentSpeedConstraints(double curvature,const geometry_msgs::msg::Twist &speed,
                                       double pose_cost,const nav_msgs::msg::Path &path,
                                       double &linear,double &sign) {
    double curve_velocity=linear,obstacle_velocity=linear;
    // Keep Nav2's regulation and approach scaling; split only the speed floors.
    Base::applyConstraints(curvature,speed,nav2_costmap_2d::FREE_SPACE,path,curve_velocity,sign);
    curve_velocity=std::max(curve_velocity,std::min(curvature_min_speed_,desired_linear_vel_));
    Base::applyConstraints(0.0,speed,pose_cost,path,obstacle_velocity,sign);
    // The obstacle branch retains endpoint deceleration, even below either floor.
    linear=std::min(curve_velocity,obstacle_velocity);
  }
  double plannedCurvature(double lookahead) const {
    return planned_curvature(indexed_,lookahead,spacing_,curvature_spacing_);
  }
  bool enabled_=true,plan_changed_=true;
  double spacing_=0.01,search_ahead_=0.5,curvature_spacing_=0.3,curvature_min_speed_=1.5;
  double last_target_progress_=0.0,handoff_tau_=0.1,last_executed_curvature_=0;
  double last_kept_prefix_m_=0,last_reference_error_m_=0;
  double motion_preference_=0,last_command_v_=0;
  nav2_msgs::msg::Costmap::ConstSharedPtr motion_global_;
  HandoffGrid motion_global_grid_,motion_local_grid_;
  std::vector<unsigned char> motion_local_data_;
  std::vector<Point> motion_footprint_;
  CurveState motion_robot_,motion_local_robot_;
  double terminal_deceleration_=0.6,terminal_response_time_=0.2,terminal_stop_margin_=0.15;
  bool handoff_pending_=false,last_preview_valid_=false,cycle_initialized_=false,last_handoff_mapped_=false;
  bool replan_changed_geometry_=false;
  std::chrono::steady_clock::time_point last_cycle_;
  IndexedPath tracking_;
  CurvatureHandoff curvature_handoff_;
  GoalPriorityBlend goal_priority_blend_;
  double goal_join_limit_=std::numeric_limits<double>::infinity(),goal_limit_cached_=0;
  std::shared_ptr<RemainingGoalsSnapshot> goal_limit_snapshot_;
  std::uint64_t goal_limit_plan_revision_=0,goal_limit_cache_revision_=0;
  std::shared_ptr<RemainingGoalsCache> remaining_goals_cache_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;
  std::shared_ptr<GlobalGridCache> global_grid_cache_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_grid_sub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_parameters_;
  std::shared_future<std::vector<rclcpp::Parameter>> planner_future_;
  std::chrono::steady_clock::time_point last_constraint_request_;
  double planning_radius_=0;
  bool planning_allow_unknown_=false,planning_constraints_ready_=false;
  std::string handoff_status_="raw_plan";
  unsigned debug_cycle_=0;
  IndexedPath indexed_;
  nav_msgs::msg::Path full_path_,control_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr samples_pub_;
};
}  // namespace racecar_multipoint_controller
PLUGINLIB_EXPORT_CLASS(racecar_multipoint_controller::MultiPointController,nav2_core::Controller)

PLUGINLIB_EXPORT_CLASS(racecar_multipoint_controller::OrderedGoalChecker,nav2_core::GoalChecker)
