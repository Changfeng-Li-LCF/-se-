// IMU ingestion never calls Python: a bounded, stamped integration history is
// copied on demand by the scan worker. No actuator or odometry publishers.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace {
struct Sample { double stamp, rate, angle; };
class ImuRuntime {
public:
  ImuRuntime(const char *topic, const char *node_name, const char *node_namespace,
             const double *axis, double bias, const char *frame,
             double max_rate, double max_gap, bool simulated)
    : bias_(bias), max_rate_(max_rate), max_gap_(max_gap), frame_(frame) {
    for (size_t i = 0; i < 3; ++i) axis_[i] = axis[i];
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_).use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", simulated)});
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace, options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_.reset(new rclcpp::executors::SingleThreadedExecutor(executor_options));
    subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      topic, rclcpp::QoS(rclcpp::KeepLast(32)).best_effort(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { receive(*msg); });
    executor_->add_node(node_);
    thread_ = std::thread([this]() {
      try { executor_->spin(); }
      catch (const std::exception &) { failed_.store(true); }
      running_.store(false);
      changed_.notify_all();
    });
  }

  ~ImuRuntime() {
    stopping_.store(true);
    changed_.notify_all();
    // Shutdown also wakes an executor that has not entered spin() yet.
    context_->shutdown("scan motion IMU receiver stopped");
    executor_->cancel();
    if (thread_.joinable()) thread_.join();
  }

  int snapshot(double *rows, double *meta) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failed_.load() || !running_.load()) return -1;
    for (size_t i = 0; i < count_; ++i) {
      const auto &s = samples_[(head_ + i) % samples_.size()];
      rows[3*i] = s.stamp; rows[3*i+1] = s.rate; rows[3*i+2] = s.angle;
    }
    meta[0] = generation_; meta[1] = accepted_; meta[2] = rejected_;
    meta[3] = resets_; meta[4] = last_callback_; meta[5] = sequence_;
    return static_cast<int>(count_);
  }

  void wait(double sequence, double seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, std::chrono::duration<double>(seconds), [this, sequence]() {
      return sequence_ != sequence || stopping_.load() || !running_.load();
    });
  }

private:
  void reset() { head_ = count_ = 0; ++generation_; ++resets_; }
  void receive(const sensor_msgs::msg::Imu &msg) {
    const double now = node_->get_clock()->now().seconds();
    // Match the scan/Python header conversion exactly at epoch-sized values.
    const double stamp = static_cast<double>(msg.header.stamp.sec) +
                         static_cast<double>(msg.header.stamp.nanosec)*1e-9;
    const auto &v = msg.angular_velocity;
    const double rate = axis_[0]*v.x + axis_[1]*v.y + axis_[2]*v.z - bias_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (now < last_callback_) reset();
      last_callback_ = now;
      ++sequence_;
      if (msg.header.frame_id != frame_ || !std::isfinite(rate) ||
          std::abs(rate) > max_rate_ || !std::isfinite(stamp) ||
          (count_ && stamp <= samples_[(head_+count_-1)%samples_.size()].stamp)) {
        ++rejected_;
      } else {
        double angle = 0.0;
        if (count_) {
          const auto old = samples_[(head_+count_-1)%samples_.size()];
          const double dt = stamp - old.stamp;
          if (dt > max_gap_) reset();
          else angle = old.angle + 0.5*(rate+old.rate)*dt;
        }
        if (count_ == samples_.size()) { head_=(head_+1)%samples_.size(); --count_; }
        samples_[(head_+count_)%samples_.size()] = {stamp, rate, angle};
        ++count_; ++accepted_;
        while (count_ > 2 && stamp-samples_[head_].stamp > 2.0) {
          head_=(head_+1)%samples_.size(); --count_;
        }
      }
    }
    changed_.notify_all();
  }
  std::array<Sample, 512> samples_{};
  std::array<double, 3> axis_{};
  size_t head_=0, count_=0;
  double bias_, max_rate_, max_gap_;
  std::string frame_;
  double generation_=0, accepted_=0, rejected_=0, resets_=0, sequence_=0, last_callback_=0;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::atomic<bool> failed_{false}, stopping_{false}, running_{true};
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;
};
}

extern "C" {
unsigned scan_motion_imu_abi() { return 1; }
void *scan_motion_imu_create(const char *topic, const char *name, const char *ns,
    const double *axis, double bias, const char *frame, double max_rate, double max_gap,
    int simulated, char *error, size_t error_size) {
  try { return new ImuRuntime(topic, name, ns, axis, bias, frame, max_rate, max_gap, simulated != 0); }
  catch (const std::exception &e) { std::snprintf(error, error_size, "%s", e.what()); return nullptr; }
}
int scan_motion_imu_snapshot(void *handle, double *rows, double *meta) {
  return static_cast<ImuRuntime *>(handle)->snapshot(rows, meta);
}
void scan_motion_imu_wait(void *handle, double sequence, double seconds) {
  static_cast<ImuRuntime *>(handle)->wait(sequence, seconds);
}
void scan_motion_imu_destroy(void *handle) { delete static_cast<ImuRuntime *>(handle); }
}
