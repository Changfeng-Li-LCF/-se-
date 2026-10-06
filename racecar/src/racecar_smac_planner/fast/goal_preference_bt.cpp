// Exploration headings are preferences. Each attempt still uses the normal
// curvature-constrained planner and its current footprint/costmap checks.
#include <cmath>
#include <chrono>
#include <memory>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace racecar_goal_preference
{
using Pose = geometry_msgs::msg::PoseStamped;

double yaw(const Pose & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

bool newer_request(const Pose & candidate, const Pose & previous)
{
  return candidate.header.stamp.sec > previous.header.stamp.sec ||
         (candidate.header.stamp.sec == previous.header.stamp.sec &&
         candidate.header.stamp.nanosec > previous.header.stamp.nanosec);
}

// Keep ordinary replanning at the configured rate, but do not delay a fresh
// goal behind the rate timer. A running planner retains its frozen candidate:
// newer messages are consumed only after that complete attempt resolves.
class PromptGoalUpdater : public BT::DecoratorNode
{
public:
  PromptGoalUpdater(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    remaining_goals_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/navigation/remaining_goals", rclcpp::QoS(1).reliable().durability_volatile());
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    std::string topic;
    node_->get_parameter_or<std::string>("goal_updater_topic", topic, "goal_update");
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    subscription_ = node_->create_subscription<Pose>(topic, rclcpp::SystemDefaultsQoS(),
      [this](const Pose::SharedPtr p) {
        // A retransmitted or reordered message cannot replace a newer request.
        if (newer_request(*p, latest_)) {latest_ = *p;}
      }, options);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("input_goal"), BT::OutputPort<Pose>("output_goal"),
      BT::InputPort<double>("hz", 10.0, "Ordinary replanning frequency")};
  }

  BT::NodeStatus tick() override
  {
    executor_.spin_some();
    Pose requested;
    double hz;
    if (!getInput("input_goal", requested) || !getInput("hz", hz) ||
      !std::isfinite(hz) || hz <= 0.) {return BT::NodeStatus::FAILURE;}
    if (newer_request(latest_, requested)) {requested = latest_;}
    const bool first = status() == BT::NodeStatus::IDLE;
    setStatus(BT::NodeStatus::RUNNING);
    const bool running = child_node_->status() == BT::NodeStatus::RUNNING;
    const auto now = std::chrono::steady_clock::now();
    if (!running && !first && !newer_request(requested, frozen_) && now < next_) {
      return BT::NodeStatus::RUNNING;
    }
    if (!running) {
      frozen_ = requested;
      setOutput("output_goal", frozen_);
    }
    const auto result = child_node_->executeTick();
    if (result == BT::NodeStatus::SUCCESS) {
      next_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1. / hz));
      nav_msgs::msg::Path path;
      Pose accepted;
      if (config().blackboard->get("path", path) && !path.poses.empty() &&
          config().blackboard->get("accepted_goal", accepted)) {
        nav_msgs::msg::Path remaining;
        remaining.header = path.header;
        remaining.poses = {accepted};
        remaining_goals_publisher_->publish(remaining);
      }
    }
    return result;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Pose>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr remaining_goals_publisher_;
  Pose latest_, frozen_;
  std::chrono::steady_clock::time_point next_{};
};

class PreferDirectGoal : public BT::SyncActionNode
{
public:
  PreferDirectGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("goal"), BT::OutputPort<Pose>("preferred_goal"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint")};
  }

  BT::NodeStatus tick() override
  {
    Pose goal;
    std::string base;
    if (!getInput("goal", goal) || !getInput("robot_base_frame", base)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y)) {
      return BT::NodeStatus::FAILURE;
    }
    const auto buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    try {
      const auto tf = buffer->lookupTransform(goal.header.frame_id, base, tf2::TimePointZero);
      const double dx = goal.pose.position.x - tf.transform.translation.x;
      const double dy = goal.pose.position.y - tf.transform.translation.y;
      if (std::hypot(dx, dy) > 1e-6) {
        const double a = std::atan2(dy, dx);
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(a/2.0);
        goal.pose.orientation.w = std::cos(a/2.0);
      }
    } catch (const tf2::TransformException &) {
      // Keep the supplied heading. ComputePathToPose remains responsible for
      // rejecting an unavailable current pose; this node never authorizes motion.
    }
    setOutput("preferred_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }
};

class GoalHeadingDifferent : public BT::ConditionNode
{
public:
  GoalHeadingDifferent(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("first"), BT::InputPort<Pose>("second")};
  }
  BT::NodeStatus tick() override
  {
    Pose a, b;
    if (!getInput("first", a) || !getInput("second", b)) {return BT::NodeStatus::FAILURE;}
    const double delta = yaw(a) - yaw(b);
    return std::abs(std::atan2(std::sin(delta), std::cos(delta))) > 1e-4 ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class GoalRequestAllowed : public BT::ConditionNode
{
public:
  GoalRequestAllowed(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("candidate_goal"), BT::InputPort<Pose>("rejected_goal")};
  }

  BT::NodeStatus tick() override
  {
    Pose candidate, rejected;
    if (!getInput("candidate_goal", candidate)) {return BT::NodeStatus::FAILURE;}
    if (!getInput("rejected_goal", rejected)) {return BT::NodeStatus::SUCCESS;}
    // Rejection is terminal for one proposal stamp. A cached GoalUpdater
    // message must not be accepted later after its requester has moved on.
    return newer_request(candidate, rejected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// A goal update is only a proposal until its path is committed by this tree.
// Keep the original request stamp even when its preferred heading falls back to
// a direct heading, so the session can match a result to one specific proposal.
class ReportRollingGoal : public BT::SyncActionNode
{
public:
  ReportRollingGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
      "/live_slam/rolling_goal_status", rclcpp::QoS(10).reliable().transient_local());
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("status"), BT::InputPort<Pose>("requested_goal"),
      BT::InputPort<Pose>("accepted_goal"), BT::BidirectionalPort<Pose>("rejected_goal")};
  }

  BT::NodeStatus tick() override
  {
    Pose requested, accepted;
    std::string status;
    if (!getInput("status", status) || !getInput("requested_goal", requested) ||
      (status != "accepted" && status != "rejected") ||
      (status == "accepted" && !getInput("accepted_goal", accepted))) {
      return BT::NodeStatus::FAILURE;
    }
    const auto finite = [](const Pose & p) {
        return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y) &&
               std::isfinite(yaw(p));
      };
    if (!finite(requested) || (status == "accepted" && !finite(accepted))) {
      return BT::NodeStatus::FAILURE;
    }
    if (status == "rejected") {
      Pose previous;
      if (!getInput("rejected_goal", previous) || newer_request(requested, previous)) {
        setOutput("rejected_goal", requested);
      }
    }
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "{\"status\":\"" << status << "\",\"request_stamp_sec\":"
        << requested.header.stamp.sec << ",\"request_stamp_nanosec\":"
        << requested.header.stamp.nanosec << ",\"request_goal\":["
        << requested.pose.position.x << ',' << requested.pose.position.y << ',' << yaw(requested) << ']';
    if (status == "accepted") {
      out << ",\"accepted_goal\":[" << accepted.pose.position.x << ','
          << accepted.pose.position.y << ',' << yaw(accepted) << ']';
    }
    out << '}';
    const auto payload = out.str();
    // Retained reliable reports survive late DDS discovery. The receiver must
    // match the original request stamp before accepting a retained report.
    // Normal 10 Hz replanning of the same committed goal needs no repeated
    // acknowledgment. A new request stamp always causes a new report.
    if (payload != last_report_) {
      std_msgs::msg::String message;
      // Timestamp only new reports; unchanged geometry/stamp stays deduplicated.
      std::ostringstream timed;
      timed.imbue(std::locale::classic());
      timed << payload.substr(0, payload.size()-1) << ",\"reported_ros_s\":"
            << std::setprecision(17) << node_->now().seconds() << '}';
      message.data = timed.str();
      publisher_->publish(message);
      last_report_ = payload;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string last_report_;
};

using Goals = std::vector<Pose>;

// The penultimate alignment point lies directly behind the final target and
// shares its heading. Keep it until its transverse plane has been crossed;
// ordinary viapoints retain the original radius-based removal policy.
bool terminal_alignment_guide(const Goals & goals)
{
  if (goals.size() != 2) {return false;}
  const auto & guide = goals.front();
  const auto & end = goals.back();
  const double angle = yaw(end);
  const double dx = end.pose.position.x - guide.pose.position.x;
  const double dy = end.pose.position.y - guide.pose.position.y;
  const double turn = yaw(guide) - angle;
  return dx*std::cos(angle) + dy*std::sin(angle) > 1e-6 &&
    std::abs(-dx*std::sin(angle) + dy*std::cos(angle)) < 1e-4 &&
    std::abs(std::atan2(std::sin(turn), std::cos(turn))) < 1e-4;
}

Goals remove_passed_marked_goals(Goals goals, const Pose & current, double radius)
{
  while (goals.size() > 1) {
    const auto & front = goals.front();
    const double dx = current.pose.position.x - front.pose.position.x;
    const double dy = current.pose.position.y - front.pose.position.y;
    if (terminal_alignment_guide(goals)) {
      const double angle = yaw(goals.back());
      if (dx*std::cos(angle) + dy*std::sin(angle) < 0.0) {break;}
    } else if (std::hypot(dx, dy) > radius) {break;}
    goals.erase(goals.begin());
  }
  return goals;
}

class RemovePassedMarkedGoals : public BT::SyncActionNode
{
public:
  RemovePassedMarkedGoals(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Goals>("input_goals"), BT::OutputPort<Goals>("output_goals"),
      BT::InputPort<double>("radius", 0.7, "Ordinary waypoint removal radius"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base frame")};
  }

  BT::NodeStatus tick() override
  {
    Goals goals;
    double radius;
    std::string frame, base;
    if (!getInput("input_goals", goals) || !getInput("radius", radius) ||
        !getInput("global_frame", frame) || !getInput("robot_base_frame", base))
    {return BT::NodeStatus::FAILURE;}
    const auto buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    try {
      const auto tf = buffer->lookupTransform(frame, base, tf2::TimePointZero);
      Pose current;
      current.pose.position.x = tf.transform.translation.x;
      current.pose.position.y = tf.transform.translation.y;
      setOutput("output_goals", remove_passed_marked_goals(std::move(goals), current, radius));
      return BT::NodeStatus::SUCCESS;
    } catch (const tf2::TransformException &) {return BT::NodeStatus::FAILURE;}
  }
};

// A retained path belongs to one request. Removing passed goals is allowed;
// changing a goal, its heading, frame or request stamp is not a cache match.
bool same_remaining_goals(const Goals & requested, const Goals & committed)
{
  if (requested.empty() || requested.size() > committed.size()) {return false;}
  const size_t offset = committed.size() - requested.size();
  for (size_t i = 0; i < requested.size(); ++i) {
    const auto & a = requested[i];
    const auto & b = committed[offset + i];
    if (a.header.frame_id != b.header.frame_id || a.header.stamp != b.header.stamp ||
      !std::isfinite(a.pose.position.x) || !std::isfinite(a.pose.position.y) ||
      !std::isfinite(yaw(a)) ||
      std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y) > 1e-6 ||
      std::abs(std::atan2(std::sin(yaw(a) - yaw(b)), std::cos(yaw(a) - yaw(b)))) > 1e-6)
    {
      return false;
    }
  }
  return true;
}

class CommitMarkedPath : public BT::SyncActionNode
{
public:
  CommitMarkedPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    remaining_goals_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/navigation/remaining_goals", rclcpp::QoS(1).reliable().durability_volatile());
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("candidate"), BT::InputPort<Goals>("goals"),
      BT::OutputPort<nav_msgs::msg::Path>("path"), BT::OutputPort<Goals>("committed_goals")};
  }
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path candidate;
    Goals goals;
    if (!getInput("candidate", candidate) || candidate.poses.empty() ||
      !getInput("goals", goals) || goals.empty()) {return BT::NodeStatus::FAILURE;}
    setOutput("committed_goals", goals);
    setOutput("path", candidate);
    nav_msgs::msg::Path remaining;
    remaining.header = candidate.header;
    remaining.poses = goals;
    remaining_goals_publisher_->publish(remaining);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr remaining_goals_publisher_;
};

class RetainValidMarkedPath : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::IsPathValid>
{
public:
  RetainValidMarkedPath(const std::string & name, const BT::NodeConfiguration & config)
  : BtServiceNode(name, config, "is_path_valid")
  {
    remaining_goals_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      "/navigation/remaining_goals", rclcpp::QoS(1).reliable().durability_volatile());
  }
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<Goals>("goals"), BT::InputPort<Goals>("committed_goals"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint")});
  }
  void on_tick() override
  {
    Goals goals, committed;
    nav_msgs::msg::Path path;
    std::string base;
    if (!getInput("goals", goals) || !getInput("committed_goals", committed) ||
      !same_remaining_goals(goals, committed) || !getInput("path", path) || path.poses.empty() ||
      !getInput("robot_base_frame", base) || path.header.frame_id.empty())
    {
      should_send_request_ = false;
      return;
    }
    try {
      const auto buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      const auto tf = buffer->lookupTransform(path.header.frame_id, base, tf2::TimePointZero);
      Pose current;
      current.header = path.header;
      current.pose.position.x = tf.transform.translation.x;
      current.pose.position.y = tf.transform.translation.y;
      current.pose.position.z = tf.transform.translation.z;
      current.pose.orientation = tf.transform.rotation;
      // IsPathValid starts at its closest pose. An exact current-pose prefix
      // prevents an 8-shaped path's later crossing from skipping the current leg.
      // This check conservatively includes passed points as well; the controller
      // continues to receive the unchanged ordered path, not this request copy.
      path.poses.insert(path.poses.begin(), current);
      request_->path = std::move(path);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node_->get_logger(), "Cannot retain marked path without robot pose: %s", e.what());
      should_send_request_ = false;
    }
  }
  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response) override
  {
    Goals goals, committed;
    if (!getInput("goals", goals) || !getInput("committed_goals", committed) ||
      !same_remaining_goals(goals, committed)) {return BT::NodeStatus::FAILURE;}
    if (!response->is_valid) {
      RCLCPP_WARN(node_->get_logger(), "Marked replan failed and the retained path is no longer valid");
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Marked replan failed; retained path checked valid, continuing tracking and replanning");
    nav_msgs::msg::Path path;
    if (getInput("path", path) && !path.poses.empty()) {
      nav_msgs::msg::Path remaining;
      remaining.header = path.header;
      remaining.poses = goals;
      remaining_goals_publisher_->publish(remaining);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr remaining_goals_publisher_;
};

}  // namespace racecar_goal_preference

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<racecar_goal_preference::RemovePassedMarkedGoals>("RemovePassedMarkedGoals");
  factory.registerNodeType<racecar_goal_preference::CommitMarkedPath>("CommitMarkedPath");
  factory.registerNodeType<racecar_goal_preference::RetainValidMarkedPath>("RetainValidMarkedPath");
  factory.registerNodeType<racecar_goal_preference::PromptGoalUpdater>("PromptGoalUpdater");
  factory.registerNodeType<racecar_goal_preference::PreferDirectGoal>("PreferDirectGoal");
  factory.registerNodeType<racecar_goal_preference::GoalHeadingDifferent>("GoalHeadingDifferent");
  factory.registerNodeType<racecar_goal_preference::GoalRequestAllowed>("GoalRequestAllowed");
  factory.registerNodeType<racecar_goal_preference::ReportRollingGoal>("ReportRollingGoal");
}
