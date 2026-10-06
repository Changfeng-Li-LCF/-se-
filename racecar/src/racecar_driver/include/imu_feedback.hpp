#pragma once
#include "closed_loop.hpp"
#include <array>
#include <string>

namespace racecar_control {
struct ImuYawSettings {
  std::string frame="IMU_link";
  // Body +Z expressed in the sensor frame; mounting assumption, not a TF lookup.
  std::array<double,3> axis{0.0,0.0,1.0};
  double bias_radps=0.0, timeout_s=0.15, filter_tau_s=0.02, max_radps=6.0;
  void validate() const {
    double norm2=0;
    for (double x:axis) {
      if (!std::isfinite(x)) throw std::invalid_argument("IMU yaw axis must be finite");
      norm2+=x*x;
    }
    if (frame.empty() || std::abs(norm2-1.0)>1e-6 || !std::isfinite(bias_radps) ||
        !std::isfinite(timeout_s) || timeout_s<=0 || timeout_s>0.5 ||
        !std::isfinite(filter_tau_s) || filter_tau_s<=0 ||
        !std::isfinite(max_radps) || max_radps<=0 || std::abs(bias_radps)>=max_radps)
      throw std::invalid_argument("Invalid IMU yaw feedback configuration");
  }
  LoopSettings sample_settings() const {
    LoopSettings p;
    p.feedback_timeout_s=timeout_s;
    p.feedback_filter_tau_s=filter_tau_s;
    p.feedback_max_yaw_rate_radps=max_radps;
    return p;
  }
};

struct ImuYawFeedback {
  FeedbackSample sample;
  double raw_w=0.0;
  uint64_t accepted=0,rejected=0;
  void reset() { sample.reset();raw_w=0; }
  bool push(const ImuYawSettings &p,const std::string &frame,double stamp,
            double ros_now,double steady_now,const std::array<double,3> &gyro,
            double covariance_0) {
    if (frame!=p.frame || !std::isfinite(covariance_0) || covariance_0<0) {
      ++rejected;return false;
    }
    double projected=0;
    for (size_t i=0;i<3;++i) {
      if (!std::isfinite(gyro[i])) { ++rejected;return false; }
      projected+=p.axis[i]*gyro[i];
    }
    if (!sample.push(p.sample_settings(),stamp,ros_now,steady_now,0,projected-p.bias_radps)) {
      ++rejected;return false;
    }
    raw_w=projected;++accepted;return true;
  }
  bool fresh(const ImuYawSettings &p,double ros_now,double steady_now) const {
    return sample.fresh(p.sample_settings(),ros_now,steady_now);
  }
};
} // namespace racecar_control
