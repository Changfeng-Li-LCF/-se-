#pragma once
#include "control_math.hpp"

namespace racecar_control {
// Feedback is vehicle velocity from localization, not motor/servo shaft telemetry.
struct LoopSettings {
  bool speed_feedback_enabled=false, yaw_rate_feedback_enabled=false;
  bool speed_feedback_confidence_enabled=true;
  double motor_kp=20.0, motor_ki=10.0;
  double speed_setpoint_mps=0.0; // 0: respect input; positive: cruise target (navigation may slow).
  double motor_integral_limit_pwm=50.0, motor_correction_limit_pwm=75.0;
  double motor_pwm_rise_rate_per_s=100.0, motor_pwm_fall_rate_per_s=200.0;
  double motor_startup_pwm_rise_rate_per_s=300.0;
  double motor_startup_pwm=1550.0, motor_startup_timeout_s=1.5;
  double motor_startup_motion_mps=0.08, motor_startup_confirm_s=0.15;
  double yaw_kp_deg_per_radps=15.0, yaw_ki_deg_per_rad=2.0;
  double yaw_integral_limit_deg=5.0, yaw_correction_limit_deg=10.0;
  double yaw_feedback_min_speed_mps=0.03;
  double feedback_timeout_s=0.30, feedback_filter_tau_s=0.08;
  // Legacy speed-limit parameter is retained for config compatibility, not sample rejection.
  double feedback_max_speed_mps=2.0, feedback_max_yaw_rate_radps=6.0;
  // Tolerate velocity noise and lateral motion of a reference point during turns.
  double feedback_lateral_noise_mps=0.20, feedback_lateral_ratio=0.35;
  double feedback_turn_lever_arm_m=0.25;
  double feedback_confidence_recovery_s=0.15;
  void validate() const {
    for (double v : {motor_kp,motor_ki,speed_setpoint_mps,motor_integral_limit_pwm,motor_correction_limit_pwm,
                     yaw_kp_deg_per_radps,yaw_ki_deg_per_rad,yaw_integral_limit_deg,
                     yaw_correction_limit_deg,yaw_feedback_min_speed_mps,feedback_timeout_s,
                     feedback_filter_tau_s,feedback_max_speed_mps,feedback_max_yaw_rate_radps,
                     motor_pwm_rise_rate_per_s,motor_pwm_fall_rate_per_s,motor_startup_pwm_rise_rate_per_s,motor_startup_pwm,
                     motor_startup_timeout_s,motor_startup_motion_mps,motor_startup_confirm_s,
                     feedback_lateral_noise_mps,feedback_lateral_ratio,feedback_turn_lever_arm_m,
                     feedback_confidence_recovery_s})
      if (!std::isfinite(v) || v<0) throw std::invalid_argument("Invalid feedback-loop setting");
    if (feedback_timeout_s<=0 || feedback_timeout_s>0.5 || feedback_filter_tau_s<=0 ||
        feedback_max_speed_mps<=0 || feedback_max_yaw_rate_radps<=0 ||
        yaw_feedback_min_speed_mps<=0 ||
        (speed_feedback_enabled && (motor_kp<=0 || motor_ki<=0 ||
          motor_pwm_rise_rate_per_s<=0 || motor_pwm_fall_rate_per_s<=0 || motor_startup_pwm_rise_rate_per_s<=0 ||
          motor_startup_pwm<500 || motor_startup_pwm>2500 || motor_startup_timeout_s>2.0 ||
          motor_startup_motion_mps<=0 || motor_startup_confirm_s<=0)) ||
        (yaw_rate_feedback_enabled && (yaw_kp_deg_per_radps<=0 || yaw_correction_limit_deg<=0)))
      throw std::invalid_argument("Feedback requires positive limits and proportional gains");
  }
  bool enabled() const { return speed_feedback_enabled || yaw_rate_feedback_enabled; }
};

inline double steering_degrees(const Calibration &c, double servo) {
  const double left=(servo-c.servo_center_pwm)/(c.servo_left_pwm-c.servo_center_pwm);
  return left>=0 ? std::clamp(left,0.0,1.0)*c.left_angle_deg :
      -std::clamp((servo-c.servo_center_pwm)/(c.servo_right_pwm-c.servo_center_pwm),0.0,1.0)*c.right_angle_deg;
}
inline uint16_t steering_pwm(const Calibration &c, double angle) {
  angle=std::clamp(angle,-c.right_angle_deg,c.left_angle_deg);
  const double value=angle>=0
      ? c.servo_center_pwm+angle/c.left_angle_deg*(c.servo_left_pwm-c.servo_center_pwm)
      : c.servo_center_pwm-angle/c.right_angle_deg*(c.servo_right_pwm-c.servo_center_pwm);
  return pwm(value,std::min(c.servo_left_pwm,c.servo_right_pwm),std::max(c.servo_left_pwm,c.servo_right_pwm));
}
struct Target {
  double v=0, w=0, steering_deg=0;
  bool valid=true;
};
inline Target twist_target(const Calibration &c,double v,double w) {
  if (!std::isfinite(v) || !std::isfinite(w)) return {0,0,0,false};
  if (std::abs(v)<c.speed_epsilon_mps) return {};
  const double angle=std::clamp(std::atan(c.wheelbase_m*(w/v))*180.0/kPi,
                                -c.right_angle_deg,c.left_angle_deg);
  if (c.speed_limit_enabled) v=std::clamp(v,-c.max_speed_mps,c.max_speed_mps);
  if (v<0 && c.motor_min_pwm>=c.motor_neutral_pwm) return {};
  // Saturating speed retains curvature; saturating steering also bounds feasible yaw demand.
  return {v,v*std::tan(angle*kPi/180.0)/c.wheelbase_m,angle,true};
}
inline Target teleop_target(const Calibration &c,double motor,double degrees) {
  const auto command=from_teleop(c,motor,degrees);
  if (!command.valid) return {0,0,0,false};
  const double difference=command.motor-c.motor_neutral_pwm;
  double v=difference/(difference>=0 ? c.forward_pwm_per_mps : c.reverse_pwm_per_mps);
  if (c.speed_limit_enabled) v=std::clamp(v,-c.max_speed_mps,c.max_speed_mps);
  if (std::abs(v)<c.speed_epsilon_mps) v=0;
  const double angle=steering_degrees(c,command.servo);
  return {v,v*std::tan(angle*kPi/180.0)/c.wheelbase_m,angle,true};
}

inline Target shared_speed_target(const Calibration &c,const LoopSettings &p,Target target,
                                  bool respect_lower_command=false) {
  if (p.speed_feedback_enabled && p.speed_setpoint_mps>0 && target.valid &&
      std::abs(target.v)>=c.speed_epsilon_mps) {
    // Navigation owns endpoint/obstacle slowing. The shared setting is its
    // cruise ceiling, never a reason to accelerate a deliberately slow command.
    // Raw-PWM keyboard input retains the existing shared running setpoint.
    const double speed=respect_lower_command ?
      std::min(std::abs(target.v),p.speed_setpoint_mps) : p.speed_setpoint_mps;
    target.v=std::copysign(speed,target.v);
    if (c.speed_limit_enabled) target.v=std::clamp(target.v,-c.max_speed_mps,c.max_speed_mps);
    target.w=target.v*std::tan(target.steering_deg*kPi/180.0)/c.wheelbase_m;
  }
  return target;
}

class BoundedPI {
 public:
  void reset() { integral_=0; }
  double integral() const { return integral_; }
  double update(double error,double dt,double ff,double low,double high,
                double kp,double ki,double integral_limit,double correction_limit) {
    const double next=std::clamp(integral_+ki*error*dt,-integral_limit,integral_limit);
    const double correction=kp*error+next;
    // Conditional integration prevents windup at either correction or actuator saturation.
    const bool pushing_high=error>0 && (correction>correction_limit || ff+correction>high);
    const bool pushing_low=error<0 && (correction< -correction_limit || ff+correction<low);
    if (!pushing_high && !pushing_low) integral_=next;
    return std::clamp(ff+std::clamp(kp*error+integral_,-correction_limit,correction_limit),low,high);
  }
 private:
  double integral_=0;
};

struct FeedbackSample {
  double stamp=0, received=0, v=0, w=0;
  double raw_v=0, lateral_v=0, lateral_limit=0, good_since=0;
  bool speed_credible=false, confidence_blocked=false;
  const char *confidence_reason="no_feedback";
  unsigned samples=0;
  void reset() { *this=FeedbackSample{}; }
  bool credible(const LoopSettings &p) const { return !p.speed_feedback_confidence_enabled || speed_credible; }
  bool push(const LoopSettings &p,double source_stamp,double ros_now,double steady_now,double speed,double yaw_rate,
            double lateral_speed=0,double quality_yaw_rate=0,bool measurement_credible=true) {
    for (double value : {source_stamp,ros_now,steady_now,speed,yaw_rate})
      if (!std::isfinite(value)) return false;
    const double age=ros_now-source_stamp;
    if (source_stamp<=0 || age< -0.05 || age>p.feedback_timeout_s ||
        std::abs(yaw_rate)>p.feedback_max_yaw_rate_radps ||
        (samples && source_stamp<=stamp)) return false;
    // Quality is independent of age: a dubious but current sample still renews freshness.
    raw_v=speed;lateral_v=std::isfinite(lateral_speed) ? lateral_speed : 0.0;
    lateral_limit=p.feedback_lateral_noise_mps+p.feedback_lateral_ratio*std::abs(speed)+
        p.feedback_turn_lever_arm_m*(std::isfinite(quality_yaw_rate) ? std::abs(quality_yaw_rate) : 0.0);
    const char *bad=nullptr;
    if (!measurement_credible) bad="velocity_covariance_untrusted";
    else if (!std::isfinite(lateral_speed) || !std::isfinite(quality_yaw_rate)) bad="invalid_velocity_quality";
    else if (std::abs(lateral_speed)>lateral_limit) bad="lateral_velocity_inconsistent";
    if (!p.speed_feedback_confidence_enabled) {
      speed_credible=true;confidence_blocked=false;good_since=0;confidence_reason="disabled";
    } else if (bad) {
      speed_credible=false;confidence_blocked=true;good_since=0;confidence_reason=bad;
    } else if (confidence_blocked) {
      if (good_since<=0 || source_stamp-stamp>p.feedback_timeout_s) good_since=source_stamp;
      speed_credible=source_stamp-good_since>=p.feedback_confidence_recovery_s;
      confidence_blocked=!speed_credible;confidence_reason=speed_credible ? "credible" : "recovering";
    } else {
      speed_credible=true;confidence_reason="credible";
    }
    const double dt=source_stamp-stamp;
    if (!samples || dt>p.feedback_timeout_s) { v=speed;w=yaw_rate;samples=1; }
    else {
      const double alpha=-std::expm1(-dt/p.feedback_filter_tau_s);
      v+=alpha*(speed-v);w+=alpha*(yaw_rate-w);samples=std::min(3u,samples+1);
    }
    stamp=source_stamp;received=steady_now;return true;
  }
  bool fresh(const LoopSettings &p,double ros_now,double steady_now) const {
    return samples>=2 && std::isfinite(ros_now) && std::isfinite(steady_now) &&
        ros_now-stamp>= -0.05 && ros_now-stamp<=p.feedback_timeout_s &&
        steady_now-received>=0 && steady_now-received<=p.feedback_timeout_s;
  }
};

// Only new, fresh post-stop measurements may shorten a timed brake pulse.
class BrakeStopConfirmation {
 public:
  void reset(double requested_steady) {
    requested_=requested_steady;low_since_=last_stamp_=0;source_=-1;
  }
  bool update(const FeedbackSample *sample,int source,const LoopSettings &p,
              double ros_now,double steady_now,double speed_threshold,double confirm_s) {
    if (!sample || !sample->fresh(p,ros_now,steady_now) || !sample->credible(p) || sample->received<=requested_) {
      low_since_=last_stamp_=0;source_=-1;return false;
    }
    if (source!=source_) { low_since_=last_stamp_=0;source_=source; }
    if (std::abs(sample->v)>speed_threshold) {
      low_since_=0;last_stamp_=sample->stamp;return false;
    }
    if (sample->stamp<=last_stamp_) return false;
    last_stamp_=sample->stamp;
    if (low_since_<=0) low_since_=sample->stamp;
    return sample->stamp-low_since_>=confirm_s;
  }
 private:
  double requested_=0,low_since_=0,last_stamp_=0;
  int source_=-1;
};

// Applied effort is the PI state; no hidden integral winds up past saturation.
class IncrementalMotorPI {
 public:
  void reset() {
    effort_=last_error_=elapsed_=motion_time_=last_target_=hold_ceiling_=0;
    direction_=0;starting_=feedback_hold_=false;
  }
  double integral_bias(const LoopSettings &p) const { return effort_-p.motor_kp*last_error_; }
  bool starting() const { return starting_; }
  bool feedback_hold() const { return feedback_hold_; }
  double update(const Calibration &c,const LoopSettings &p,double target_v,double measured_v,double dt,
                bool feedback_credible=true) {
    const int direction=target_v>0 ? 1 : -1;
    const double limit=direction>0 ? c.motor_max_pwm-c.motor_neutral_pwm : c.motor_neutral_pwm-c.motor_min_pwm;
    const double speed=direction*measured_v;
    const double error=std::abs(target_v)-speed;
    const double motion_threshold=std::min(p.motor_startup_motion_mps,std::abs(target_v)*0.5);
    const double startup_effort=std::clamp(p.motor_startup_pwm-c.motor_neutral_pwm,0.0,limit);
    if (direction_!=direction) {
      reset();direction_=direction;last_error_=error;last_target_=std::abs(target_v);
      // Only forward startup PWM is calibrated. Reverse uses PI if enabled.
      starting_=direction>0 && speed<motion_threshold && p.motor_startup_timeout_s>0;
    }
    if (!feedback_credible) {
      if (!feedback_hold_) hold_ceiling_=effort_;
      // No extra throttle or integration from dubious velocity. Commanded slowing still applies.
      hold_ceiling_=std::clamp(hold_ceiling_-p.motor_kp*std::max(0.0,last_target_-std::abs(target_v)),0.0,limit);
      effort_=std::max(hold_ceiling_,effort_-p.motor_pwm_fall_rate_per_s*dt);
      feedback_hold_=true;last_target_=std::abs(target_v);motion_time_=0;
      return c.motor_neutral_pwm+direction*effort_;
    }
    // Rebase the proportional difference on recovery; retain effort without a catch-up jump.
    if (feedback_hold_) { last_error_=error;feedback_hold_=false; }
    last_target_=std::abs(target_v);
    double requested;
    if (starting_) {
      elapsed_+=dt;
      // Do not hand a high-speed start to PI before its calibrated PWM is applied.
      motion_time_=speed>=motion_threshold && effort_>=startup_effort-0.5 ? motion_time_+dt : 0;
      if (motion_time_>=p.motor_startup_confirm_s || elapsed_>=p.motor_startup_timeout_s || error<=0)
        starting_=false;
    }
    if (starting_) requested=startup_effort;
    else requested=effort_+p.motor_kp*(error-last_error_)+p.motor_ki*error*dt;
    requested=std::clamp(requested,0.0,limit);
    const double rise_rate=starting_ ? p.motor_startup_pwm_rise_rate_per_s : p.motor_pwm_rise_rate_per_s;
    effort_=std::clamp(requested,std::max(0.0,effort_-p.motor_pwm_fall_rate_per_s*dt),
                                 std::min(limit,effort_+rise_rate*dt));
    last_error_=error;
    return c.motor_neutral_pwm+direction*effort_;
  }
 private:
  double effort_=0,last_error_=0,elapsed_=0,motion_time_=0,last_target_=0,hold_ceiling_=0;
  int direction_=0;
  bool starting_=false,feedback_hold_=false;
};

// Follow requested curvature at measured speed, including speed-loop transients.
inline double feedback_yaw_target(const Calibration &c,const LoopSettings &p,
                                  const Target &target,double measured_v) {
  (void)p;
  return measured_v*std::tan(target.steering_deg*kPi/180.0)/c.wheelbase_m;
}

class VehicleLoops {
 public:
  void reset() { motor_.reset();yaw_.reset(); }
  double motor_integral(const LoopSettings &p) const { return motor_.integral_bias(p); }
  bool motor_starting() const { return motor_.starting(); }
  bool motor_feedback_hold() const { return motor_.feedback_hold(); }
  double yaw_integral() const { return yaw_.integral(); }
  Command update(const Calibration &c,const LoopSettings &p,const Target &target,
                 double measured_v,double measured_w,double dt,bool feedback_credible=true) {
    if (!target.valid || !std::isfinite(measured_v) || !std::isfinite(measured_w) ||
        !std::isfinite(dt) || dt<=0 || dt>0.2) { reset();return neutral(c,false); }
    if (std::abs(target.v)<c.speed_epsilon_mps) {
      reset();return {neutral(c).motor,steering_pwm(c,target.steering_deg),true};
    }
    const double ff=c.motor_neutral_pwm+target.v*(target.v>=0 ? c.forward_pwm_per_mps : c.reverse_pwm_per_mps);
    // Feedback may coast at neutral; it must never turn an overspeed correction into reverse drive.
    const double low=target.v>0 ? c.motor_neutral_pwm : c.motor_min_pwm;
    const double high=target.v>0 ? c.motor_max_pwm : c.motor_neutral_pwm;
    const double motor=p.speed_feedback_enabled ? motor_.update(c,p,target.v,measured_v,dt,feedback_credible) : std::clamp(ff,low,high);
    double angle=target.steering_deg;
    if (p.yaw_rate_feedback_enabled && feedback_credible && std::abs(measured_v)>=p.yaw_feedback_min_speed_mps && measured_v*target.v>0) {
      const double error=(feedback_yaw_target(c,p,target,measured_v)-measured_w)*(target.v>0 ? 1 : -1);
      angle=yaw_.update(error,dt,angle,-c.right_angle_deg,c.left_angle_deg,
          p.yaw_kp_deg_per_radps,p.yaw_ki_deg_per_rad,p.yaw_integral_limit_deg,p.yaw_correction_limit_deg);
    } else yaw_.reset(); // An Ackermann car cannot yaw in place; do not integrate while stationary.
    return {pwm(motor,low,high),steering_pwm(c,angle),true};
  }
 private:
  IncrementalMotorPI motor_;
  BoundedPI yaw_;
};
} // namespace racecar_control
