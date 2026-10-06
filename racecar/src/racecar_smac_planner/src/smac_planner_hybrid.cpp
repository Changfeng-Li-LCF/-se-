// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include "angles/angles.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

#include "Eigen/Core"
#include "racecar_smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace racecar_smac_planner
{

using namespace std::chrono;  // NOLINT
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace
{
bool validTerminalAlignmentReference(const std::vector<double> & reference)
{
  return reference.empty() || (reference.size() == 4 && reference[3] >= 0.0 &&
    std::all_of(reference.begin(), reference.end(), [](double v) {return std::isfinite(v);}));
}

// Optional terminal geometry, not an additional navigation goal. Preserve the
// existing prefix and selected endpoint; finish all turning before a straight tail.
bool makeTerminalApproach(
  const nav_msgs::msg::Path & path, size_t anchor, double approach,
  double radius, double step, double removed_length, nav_msgs::msg::Path & candidate)
{
  if (anchor == 0 || anchor + 1 >= path.poses.size() || approach <= 0.0 ||
    radius <= 0.0 || step <= 0.0 || removed_length <= approach)
  {
    return false;
  }
  const auto & end = path.poses.back().pose;
  const auto & connection = path.poses[anchor].pose;
  const double yaw = tf2::getYaw(end.orientation);
  const double gx = end.position.x - approach * std::cos(yaw);
  const double gy = end.position.y - approach * std::sin(yaw);
  // Never return to an approach plane already passed by the connection.
  if ((gx - connection.position.x) * std::cos(yaw) +
    (gy - connection.position.y) * std::sin(yaw) <= step)
  {
    return false;
  }
  auto space = std::make_shared<ompl::base::DubinsStateSpace>(radius, false);
  ompl::base::ScopedState<> from(space), to(space), sample(space);
  from[0] = connection.position.x;
  from[1] = connection.position.y;
  from[2] = tf2::getYaw(connection.orientation);
  to[0] = gx;
  to[1] = gy;
  to[2] = yaw;
  auto curve = space->dubins(from(), to());
  const double length = radius * curve.length();
  // Do not use a loop to manufacture early alignment. The straight tail does
  // not contribute to the detour allowance for the connector.
  if (!std::isfinite(length) || length <= 0.0 ||
    length > 2.0 * (removed_length - approach) ||
    curve.type_[1] != ompl::base::DubinsStateSpace::DUBINS_STRAIGHT)
  {
    return false;
  }
  double total_turn = 0.0;
  for (size_t k = 0; k < 3; ++k) {
    if (curve.type_[k] != ompl::base::DubinsStateSpace::DUBINS_STRAIGHT) {
      total_turn += curve.length_[k];
    }
  }
  if (total_turn > M_PI) {return false;}

  candidate.header = path.header;
  candidate.poses.assign(path.poses.begin(), path.poses.begin() + anchor + 1);
  const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
  bool first_time = false;  // Reuse the Dubins solution while sampling it.
  for (int k = 1; k <= samples; ++k) {
    space->interpolate(from(), to(), static_cast<double>(k) / samples,
      first_time, curve, sample());
    auto pose = path.poses[anchor];
    pose.pose.position.x = sample[0];
    pose.pose.position.y = sample[1];
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(sample[2]);
    candidate.poses.push_back(pose);
  }
  // Snap the junction and endpoint exactly; no later smoother may bend this tail.
  candidate.poses.back().pose.position.x = gx;
  candidate.poses.back().pose.position.y = gy;
  candidate.poses.back().pose.orientation = end.orientation;
  const int tail_samples = std::max(1, static_cast<int>(std::ceil(approach / step)));
  for (int k = 1; k <= tail_samples; ++k) {
    auto pose = path.poses.back();
    const double fraction = static_cast<double>(k) / tail_samples;
    pose.pose.position.x = gx + fraction * (end.position.x - gx);
    pose.pose.position.y = gy + fraction * (end.position.y - gy);
    candidate.poses.push_back(pose);
  }
  candidate.poses.back() = path.poses.back();
  return true;
}

}  // namespace

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  _heading_preferences.clear();

  RCLCPP_INFO(_logger, "Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", _max_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".smooth_path", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".smooth_path", smooth_path);

  // Saved configuration; changing the preferred radius requires a restart.
  rcl_interfaces::msg::ParameterDescriptor preferred_desc;
  preferred_desc.read_only = true;
  if (!node->has_parameter(name + ".smoother.preferred_turning_radius")) {
    node->declare_parameter(name + ".smoother.preferred_turning_radius", 0.0, preferred_desc);
  }
  node->get_parameter(name + ".smoother.preferred_turning_radius", _preferred_smoothing_radius);
  if (!std::isfinite(_preferred_smoothing_radius) || _preferred_smoothing_radius < 0.0) {
    throw std::invalid_argument("Invalid preferred smoothing radius");
  }

  rcl_interfaces::msg::ParameterDescriptor heading_desc;
  heading_desc.read_only = true;
  if (!node->has_parameter(name + ".goal_heading_tolerance_deg")) {
    node->declare_parameter(name + ".goal_heading_tolerance_deg", 0.0, heading_desc);
  }
  const double heading_degrees = node->get_parameter(name + ".goal_heading_tolerance_deg").as_double();
  if (!std::isfinite(heading_degrees) || heading_degrees < 0.0 || heading_degrees > 180.0) {
    throw std::invalid_argument("Invalid planning goal heading tolerance");
  }
  _goal_heading_tolerance_rad = heading_degrees * M_PI / 180.0;
  RCLCPP_INFO(_logger, "%s: planning goal heading tolerance=%.1f deg", name.c_str(), heading_degrees);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".terminal_alignment_reference", rclcpp::ParameterValue(std::vector<double>{}));
  node->get_parameter(name + ".terminal_alignment_reference", _terminal_alignment_reference);
  if (!validTerminalAlignmentReference(_terminal_alignment_reference)) {
    throw std::invalid_argument("Invalid terminal alignment reference");
  }

  // Historical parameter name: one tolerance now applies to search, returned
  // paths and the startup audit; the costmap's physical body stays unchanged.
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".final_path_collision_tolerance_m", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".final_path_collision_stop_enabled", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_planning_time", _max_planning_time);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
  _motion_model = fromString(_motion_model_for_search);
  if (_motion_model == MotionModel::UNKNOWN) {
    RCLCPP_WARN(
      _logger,
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    RCLCPP_INFO(
      _logger, "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    RCLCPP_INFO(
      _logger,
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(node, name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
    _wide_smoother.reset();
    if (_preferred_smoothing_radius > _minimum_turning_radius_global_coords) {
      _wide_smoother = std::make_unique<Smoother>(params);
      _wide_smoother->initialize(_preferred_smoothing_radius);
    }
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->on_configure(
      node, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));
}

void SmacPlannerHybrid::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _wide_smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
}

nav_msgs::msg::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Use the same outline for input/search and returned-path checks.
  auto footprint = _costmap_ros->getRobotFootprint();
  const bool use_radius = _costmap_ros->getUseRadius();
  const double collision_tolerance = std::max(0.0,
    _node.lock()->get_parameter(_name + ".final_path_collision_tolerance_m").as_double());
  if (!use_radius && collision_tolerance > 0.0) {
    for (auto & point : footprint) {
      point.x = std::copysign(std::max(0.0, std::abs(point.x) - collision_tolerance), point.x);
      point.y = std::copysign(std::max(0.0, std::abs(point.y) - collision_tolerance), point.y);
    }
  }
  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    footprint, use_radius, findCircumscribedCost(_costmap_ros), collision_tolerance > 0.0);
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw std::runtime_error("Start pose is out of costmap!");
  }

  double orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  const auto start_bin = static_cast<unsigned int>(orientation_bin);
  _a_star->setStart(mx, my, start_bin);
  _collision_checker.setStartPose(mx, my, start_bin,
    start.pose.position.x, start.pose.position.y, !use_radius && collision_tolerance > 0.0);

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw std::runtime_error("Goal pose is out of costmap!");
  }
  orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  std::vector<unsigned int> accepted_headings{static_cast<unsigned int>(orientation_bin)};
  if (_goal_heading_tolerance_rad > 0.0) {
    const double requested_yaw = tf2::getYaw(goal.pose.orientation);
    for (unsigned int bin = 0; bin < _angle_quantizations; ++bin) {
      const double delta = angles::shortest_angular_distance(requested_yaw, bin * _angle_bin_size);
      if (bin != accepted_headings.front() && std::abs(delta) <= _goal_heading_tolerance_rad + 1e-9) {
        accepted_headings.push_back(bin);
      }
    }
  }
  const auto same_goal = [&goal](const HeadingPreference & previous) {
      return previous.requested.header.frame_id == goal.header.frame_id &&
        std::hypot(previous.requested.pose.position.x - goal.pose.position.x,
          previous.requested.pose.position.y - goal.pose.position.y) < 1e-4 &&
        std::abs(angles::shortest_angular_distance(
          tf2::getYaw(previous.requested.pose.orientation), tf2::getYaw(goal.pose.orientation))) < 1e-4;
    };
  const auto previous_heading = std::find_if(
    _heading_preferences.begin(), _heading_preferences.end(), same_goal);
  if (previous_heading != _heading_preferences.end()) {
    const auto preferred = std::min_element(accepted_headings.begin(), accepted_headings.end(),
      [&](unsigned int a, unsigned int b) {
        return std::abs(angles::shortest_angular_distance(a * _angle_bin_size,
          previous_heading->selected_yaw)) <
          std::abs(angles::shortest_angular_distance(b * _angle_bin_size,
          previous_heading->selected_yaw));
      });
    std::rotate(accepted_headings.begin(), preferred, preferred + 1);
  }
  _a_star->setGoalAngles(mx, my, accepted_headings);

  // Setup message
  nav_msgs::msg::Path plan;
  plan.header.stamp = _clock->now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    RCLCPP_WARN(
      _logger,
      "%s: failed to create plan, %s; start=(%.3f, %.3f, %.1f deg) "
      "goal=(%.3f, %.3f, %.1f deg) preferred=%.1f deg radius=%.3f iterations=%d elapsed_ms=%.1f.",
      _name.c_str(), error.c_str(), start.pose.position.x, start.pose.position.y,
      tf2::getYaw(start.pose.orientation) * 180.0 / M_PI,
      goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI,
      accepted_headings.front() * _angle_bin_size * 180.0 / M_PI,
      _minimum_turning_radius_global_coords, num_iterations,
      duration<double, std::milli>(steady_clock::now() - a).count());
    return plan;
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }
  if (!use_radius && collision_tolerance > 0.0 && !plan.poses.empty()) {
    // Match the position audited at search entry. Final validation still checks
    // every connector from this actual pose to the remaining search states.
    plan.poses.front().pose = start.pose;
  }

  // Publish raw path for debug
  if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher->publish(plan);
  }

  // Validate the returned world poses, including the motion between poses.
  // Search checks the footprint, but the smoother only checks center costs.
  const double possible_cost = findCircumscribedCost(_costmap_ros);
  double footprint_radius = 0.0;
  for (const auto & point : footprint) {
    footprint_radius = std::max(footprint_radius, std::hypot(point.x, point.y));
  }
  const auto footprint_path_clear = [&](const nav_msgs::msg::Path & candidate) {
      const auto pose_clear = [&](double x, double y, double yaw) {
          unsigned int cx, cy;
          if (!costmap->worldToMap(x, y, cx, cy)) {
            return false;
          }
          const auto center_cost = costmap->getCost(cx, cy);
          if (use_radius) {
            return center_cost < INSCRIBED || (center_cost == UNKNOWN && _allow_unknown);
          }
          if (possible_cost > 0.0 && center_cost < possible_cost) {
            return true;
          }
          if ((collision_tolerance == 0.0 && center_cost == INSCRIBED) ||
            center_cost == OCCUPIED ||
            (center_cost == UNKNOWN && !_allow_unknown))
          {
            return false;
          }
          // Match the search/startup audit's orientation bins, without snapping
          // the returned world position to a different costmap cell center.
          const double angle = std::round(yaw / _angle_bin_size) * _angle_bin_size;
          const double edge_cost = _collision_checker.footprintCostAtPose(
            x, y, angle, footprint);
          return edge_cost < OCCUPIED || (edge_cost == UNKNOWN && _allow_unknown);
        };
      if (candidate.poses.empty()) {
        return false;
      }
      auto previous = start.pose;
      double previous_yaw = tf2::getYaw(previous.orientation);
      if (!pose_clear(previous.position.x, previous.position.y, previous_yaw)) {
        return false;
      }
      for (const auto & stamped : candidate.poses) {
        const auto & next = stamped.pose;
        const double next_yaw = tf2::getYaw(next.orientation);
        const double turn = std::atan2(
          std::sin(next_yaw - previous_yaw), std::cos(next_yaw - previous_yaw));
        const double dx = next.position.x - previous.position.x;
        const double dy = next.position.y - previous.position.y;
        const int samples = std::max(1, static_cast<int>(std::ceil(
            (std::hypot(dx, dy) + std::abs(turn) * footprint_radius) /
            (costmap->getResolution() * 0.5))));
        for (int i = 1; i <= samples; ++i) {
          const double t = static_cast<double>(i) / samples;
          if (!pose_clear(previous.position.x + t * dx, previous.position.y + t * dy,
            previous_yaw + t * turn))
          {
            return false;
          }
        }
        previous = next;
        previous_yaw = next_yaw;
      }
      return true;
    };
  const auto raw_plan = plan;
  // Only the final requested pose gets an approach preference. Keep the
  // selected feasible terminal state and heading; intermediate goals are unchanged.
  const auto & reference = _terminal_alignment_reference;
  const bool terminal = reference.size() == 4 && reference[3] > 0.0 &&
    std::hypot(goal.pose.position.x - reference[0], goal.pose.position.y - reference[1]) < 1e-6 &&
    std::abs(angles::shortest_angular_distance(tf2::getYaw(goal.pose.orientation), reference[2])) < 1e-6;
  const double approach_distance = terminal ? reference[3] : 0.0;
  const double terminal_yaw = tf2::getYaw(plan.poses.back().pose.orientation);
  const auto path_cost = [this, costmap, footprint, use_radius, possible_cost,
      approach_distance, terminal_yaw](const nav_msgs::msg::Path & candidate) {
      double score = 0.0;
      double remaining = 0.0;
      if (approach_distance > 0.0) {
        for (size_t k = 1; k < candidate.poses.size(); ++k) {
          const auto & a = candidate.poses[k - 1].pose.position;
          const auto & b = candidate.poses[k].pose.position;
          remaining += std::hypot(b.x - a.x, b.y - a.y);
        }
      }
      for (size_t k = 1; k < candidate.poses.size(); ++k) {
        const auto & a = candidate.poses[k - 1].pose;
        const auto & b = candidate.poses[k].pose;
        const double dx = b.position.x - a.position.x;
        const double dy = b.position.y - a.position.y;
        const double length = std::hypot(dx, dy);
        const double yaw = tf2::getYaw(a.orientation);
        const double turn = angles::shortest_angular_distance(yaw, tf2::getYaw(b.orientation));
        if (approach_distance > 0.0 && length > 1e-6) {
          // Integrate only the overlap with the final approach arc length.
          // This bounded cost prefers earlier alignment, never rejects a path
          // or asks the car to return to a guide it has already passed.
          const double tail = std::clamp(
            approach_distance - std::max(0.0, remaining - length), 0.0, length);
          const double heading = yaw + turn * (1.0 - 0.5 * tail / length);
          score += tail * std::max(1.0f, _search_info.non_straight_penalty) *
            (1.0 - std::cos(heading - terminal_yaw));
          remaining = std::max(0.0, remaining - length);
        }
        // Preserve the existing preferred wide radius in the objective. A
        // shorter tight arc must not win solely by saving a little distance.
        if (length > 1e-6 && _preferred_smoothing_radius > 0.0) {
          const double excess = std::max(0.0,
            _preferred_smoothing_radius * std::abs(turn) / length - 1.0);
          score += length * std::max(0.0f, _search_info.non_straight_penalty - 1.0f) *
            excess * excess;
        }
        const int samples = std::max(1, static_cast<int>(std::ceil(
            length / (costmap->getResolution() * 0.5))));
        for (int i = 0; i < samples; ++i) {
          const double t = (i + 0.5) / samples;
          const double x = a.position.x + t * dx, y = a.position.y + t * dy;
          unsigned int mx, my;
          if (!costmap->worldToMap(x, y, mx, my)) {
            return std::numeric_limits<double>::infinity();
          }
          double cost = costmap->getCost(mx, my);
          if (!use_radius && (possible_cost <= 0.0 || cost >= possible_cost)) {
            const double angle = std::round((yaw + t * turn) / _angle_bin_size) * _angle_bin_size;
            cost = std::max(cost,
              _collision_checker.footprintCostAtPose(x, y, angle, footprint));
          }
          score += length / samples *
            (1.0 + _search_info.cost_penalty * cost / 252.0);
        }
      }
      return score;
    };
  if (_smoother) {_smoother->setPathCost(path_cost);}
  if (_wide_smoother) {_wide_smoother->setPathCost(path_cost);}

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  // Leave 20 ms of the existing budget for the linear footprint scan/fallback.
  // There is no extra planning attempt, service call, sleep, or longer deadline.
  double time_remaining = std::max(
    0.0, _max_planning_time - static_cast<double>(time_span.count()) - 0.020);

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  if (_smoother && num_iterations > 1 && time_remaining > 0.0) {
    _smoother->smooth(plan, costmap, time_remaining);
  }
  if (!footprint_path_clear(plan)) {
    if (_smoother && footprint_path_clear(raw_plan)) {
      RCLCPP_WARN(_logger, "%s: smoothed path footprint collision; using valid raw path.",
        _name.c_str());
      plan = raw_plan;
    } else if (_node.lock()->get_parameter(
        _name + ".final_path_collision_stop_enabled").as_bool())
    {
      RCLCPP_WARN(_logger, "%s: path footprint collision on the current costmap.",
        _name.c_str());
      plan.poses.clear();
    } else {
      // Preserve the search's obstacle checks and discard unsafe smoothing.
      // Only the additional returned-path audit may no longer abort the route.
      RCLCPP_WARN_THROTTLE(_logger, *_node.lock()->get_clock(), 2000,
        "%s: returned-path collision stop disabled; retaining raw search path.",
        _name.c_str());
      plan = raw_plan;
    }
  }

  // Wider endpoint arcs must not depend on convergence of another full-path
  // smoothing pass, or on the other endpoint's wider arc being collision-free.
  // Audit each independently; apply the upcoming goal arc last so an accepted
  // start connection cannot replace it with a sharper goal connection.
  if (_wide_smoother && plan.poses.size() >= 3) {
    const auto same_endpoint = [](const geometry_msgs::msg::Pose & before,
      const geometry_msgs::msg::Pose & after) {
        return std::hypot(after.position.x - before.position.x,
          after.position.y - before.position.y) < 1e-6 &&
          std::abs(angles::shortest_angular_distance(tf2::getYaw(before.orientation),
            tf2::getYaw(after.orientation))) < 1e-6;
      };
    const auto try_boundary = [&](bool at_start) {
        auto candidate = plan;
        if (!_wide_smoother->smoothBoundary(candidate, costmap, at_start)) {
          return "unchanged_or_unavailable";
        }
        if (!same_endpoint(plan.poses.front().pose, candidate.poses.front().pose) ||
          !same_endpoint(plan.poses.back().pose, candidate.poses.back().pose))
        {
          return "endpoint_mismatch";
        }
        if (!footprint_path_clear(candidate)) {
          return "footprint_collision";
        }
        if (path_cost(candidate) > path_cost(plan) + 1e-6) {
          return "higher_combined_cost";
        }
        plan = std::move(candidate);
        return "accepted";
      };
    const char * wide_start = try_boundary(true);
    const char * wide_end = try_boundary(false);
    RCLCPP_INFO_THROTTLE(_logger, *_clock, 1000,
      "%s: wider endpoint arcs radius=%.2f start=%s end=%s goal=(%.2f, %.2f)",
      _name.c_str(), _preferred_smoothing_radius, wide_start, wide_end,
      goal.pose.position.x, goal.pose.position.y);
  }


  // Prefer an aligned straight terminal approach over a path which keeps
  // turning inside the slowdown zone. This is local geometry only: no extra
  // A* search, service call, hard guide, or change to arrival tolerances.
  if (terminal && plan.poses.size() >= 3) {
    const auto & end = plan.poses.back().pose;
    const double hx = std::cos(terminal_yaw), hy = std::sin(terminal_yaw);
    const double start_distance = (end.position.x - start.pose.position.x) * hx +
      (end.position.y - start.pose.position.y) * hy;
    const double step = costmap->getResolution() * 0.5;
    const char * status = "retained_feasible_path";
    double selected_radius = 0.0;
    // Do not steer back toward a virtual approach plane after passing it.
    if (start_distance > approach_distance + step) {
      std::vector<double> remaining(plan.poses.size(), 0.0);
      for (size_t k = plan.poses.size() - 1; k > 0; --k) {
        const auto & a = plan.poses[k - 1].pose.position;
        const auto & b = plan.poses[k].pose.position;
        remaining[k - 1] = remaining[k] + std::hypot(b.x - a.x, b.y - a.y);
      }
      std::vector<double> radii{std::max(
          _preferred_smoothing_radius, static_cast<double>(_minimum_turning_radius_global_coords))};
      if (radii.front() > _minimum_turning_radius_global_coords + 1e-6) {
        radii.push_back(_minimum_turning_radius_global_coords);
      }
      nav_msgs::msg::Path best;
      double best_cost = std::numeric_limits<double>::infinity();
      // At most six attachment candidates, sampled at half a costmap cell.
      for (const double radius : radii) {
        std::vector<size_t> anchors;
        for (const double extra : {radius, 2.0 * radius, M_PI * radius}) {
          size_t anchor = plan.poses.size() - 2;
          while (anchor > 0 && remaining[anchor] < approach_distance + extra) {--anchor;}
          if (anchor == 0) {anchor = 1;}
          if (std::find(anchors.begin(), anchors.end(), anchor) != anchors.end()) {continue;}
          anchors.push_back(anchor);
          nav_msgs::msg::Path candidate;
          if (!makeTerminalApproach(plan, anchor, approach_distance, radius, step,
            remaining[anchor], candidate) || !footprint_path_clear(candidate))
          {
            continue;
          }
          const double score = path_cost(candidate);
          if (score < best_cost) {
            best_cost = score;
            best = std::move(candidate);
            selected_radius = radius;
          }
        }
      }
      if (!best.poses.empty()) {
        plan = std::move(best);
        status = "straight_tail_accepted";
      }
    } else {
      status = "approach_plane_already_passed";
    }
    RCLCPP_INFO_THROTTLE(_logger, *_clock, 1000,
      "%s: terminal alignment=%s straight_tail=%.2f radius=%.2f heading=%.1f deg",
      _name.c_str(), status, approach_distance, selected_radius, terminal_yaw * 180.0 / M_PI);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  if (_goal_heading_tolerance_rad > 0.0 && !plan.poses.empty()) {
    RCLCPP_INFO_THROTTLE(_logger, *_clock, 1000,
      "%s: goal heading requested=%.1f selected=%.1f tolerance=%.1f deg states=%zu goal=(%.2f, %.2f)",
      _name.c_str(), tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI,
      tf2::getYaw(plan.poses.back().pose.orientation) * 180.0 / M_PI,
      _goal_heading_tolerance_rad * 180.0 / M_PI, accepted_headings.size(),
      goal.pose.position.x, goal.pose.position.y);
  }
  if (!plan.poses.empty()) {
    HeadingPreference preference{goal, tf2::getYaw(plan.poses.back().pose.orientation)};
    const auto old = std::find_if(_heading_preferences.begin(), _heading_preferences.end(), same_goal);
    if (old != _heading_preferences.end()) {
      *old = preference;
    } else {
      // Bounded storage also covers continually changing exploration goals.
      if (_heading_preferences.size() >= 32) {_heading_preferences.erase(_heading_preferences.begin());}
      _heading_preferences.push_back(std::move(preference));
    }
  }
  return plan;
}

rcl_interfaces::msg::SetParametersResult
SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  bool reinit_collision_checker = false;
  bool reinit_a_star = false;
  bool reinit_downsampler = false;
  bool reinit_smoother = false;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (name == _name + ".terminal_alignment_reference") {
      if (type != ParameterType::PARAMETER_DOUBLE_ARRAY ||
        !validTerminalAlignmentReference(parameter.as_double_array()))
      {
        result.successful = false;
        result.reason = "Terminal reference must be empty or finite [x, y, yaw, nonnegative distance]";
        return result;
      }
      _terminal_alignment_reference = parameter.as_double_array();
      continue;
    }
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".max_planning_time") {
        reinit_a_star = true;
        _max_planning_time = parameter.as_double();
      } else if (name == _name + ".tolerance") {
        _tolerance = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".lookup_table_size") {
        reinit_a_star = true;
        _lookup_table_size = parameter.as_double();
      } else if (name == _name + ".minimum_turning_radius") {
        reinit_a_star = true;
        if (_smoother) {
          reinit_smoother = true;
        }
        _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".reverse_penalty") {
        reinit_a_star = true;
        _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".change_penalty") {
        reinit_a_star = true;
        _search_info.change_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".non_straight_penalty") {
        reinit_a_star = true;
        _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".cost_penalty") {
        reinit_a_star = true;
        _search_info.cost_penalty = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_ratio") {
        reinit_a_star = true;
        _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
      } else if (name == _name + ".analytic_expansion_max_length") {
        reinit_a_star = true;
        _search_info.analytic_expansion_max_length =
          static_cast<float>(parameter.as_double()) / _costmap->getResolution();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".downsample_costmap") {
        reinit_downsampler = true;
        _downsample_costmap = parameter.as_bool();
      } else if (name == _name + ".allow_unknown") {
        reinit_a_star = true;
        _allow_unknown = parameter.as_bool();
      } else if (name == _name + ".cache_obstacle_heuristic") {
        reinit_a_star = true;
        _search_info.cache_obstacle_heuristic = parameter.as_bool();
      } else if (name == _name + ".smooth_path") {
        if (parameter.as_bool()) {
          reinit_smoother = true;
        } else {
          _smoother.reset();
          _wide_smoother.reset();
        }
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".downsampling_factor") {
        reinit_a_star = true;
        reinit_downsampler = true;
        _downsampling_factor = parameter.as_int();
      } else if (name == _name + ".max_iterations") {
        reinit_a_star = true;
        _max_iterations = parameter.as_int();
        if (_max_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "maximum iteration selected as <= 0, "
            "disabling maximum iterations.");
          _max_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".max_on_approach_iterations") {
        reinit_a_star = true;
        _max_on_approach_iterations = parameter.as_int();
        if (_max_on_approach_iterations <= 0) {
          RCLCPP_INFO(
            _logger, "On approach iteration selected as <= 0, "
            "disabling tolerance and on approach iterations.");
          _max_on_approach_iterations = std::numeric_limits<int>::max();
        }
      } else if (name == _name + ".angle_quantization_bins") {
        reinit_collision_checker = true;
        reinit_a_star = true;
        int angle_quantizations = parameter.as_int();
        _angle_bin_size = 2.0 * M_PI / angle_quantizations;
        _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".motion_model_for_search") {
        reinit_a_star = true;
        _motion_model = fromString(parameter.as_string());
        if (_motion_model == MotionModel::UNKNOWN) {
          RCLCPP_WARN(
            _logger,
            "Unable to get MotionModel search type. Given '%s', "
            "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
            _motion_model_for_search.c_str());
        }
      }
    }
  }

  // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
  if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    auto node = _node.lock();

    // Re-Initialize A* template
    if (reinit_a_star) {
      _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
      _a_star->initialize(
        _allow_unknown,
        _max_iterations,
        _max_on_approach_iterations,
        _max_planning_time,
        _lookup_table_dim,
        _angle_quantizations);
    }

    // Re-Initialize costmap downsampler
    if (reinit_downsampler) {
      if (_downsample_costmap && _downsampling_factor > 1) {
        std::string topic_name = "downsampled_costmap";
        _costmap_downsampler = std::make_unique<CostmapDownsampler>();
        _costmap_downsampler->on_configure(
          node, _global_frame, topic_name, _costmap, _downsampling_factor);
      }
    }

    // Re-Initialize collision checker
    if (reinit_collision_checker) {
      _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
      _collision_checker.setFootprint(
        _costmap_ros->getRobotFootprint(),
        _costmap_ros->getUseRadius(),
        findCircumscribedCost(_costmap_ros));
    }

    // Re-Initialize smoother
    if (reinit_smoother) {
      SmootherParams params;
      params.get(node, _name);
      _smoother = std::make_unique<Smoother>(params);
      _smoother->initialize(_minimum_turning_radius_global_coords);
      _wide_smoother.reset();
      if (_preferred_smoothing_radius > _minimum_turning_radius_global_coords) {
        _wide_smoother = std::make_unique<Smoother>(params);
        _wide_smoother->initialize(_preferred_smoothing_radius);
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace racecar_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_smac_planner::SmacPlannerHybrid, nav2_core::GlobalPlanner)
