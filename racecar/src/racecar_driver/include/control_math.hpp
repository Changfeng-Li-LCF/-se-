#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace racecar_control {
constexpr double kPi = 3.14159265358979323846;
struct Calibration {
  bool speed_limit_enabled = true;
  double wheelbase_m = 0.0, left_angle_deg = 0.0, right_angle_deg = 0.0;
  double servo_center_pwm = 1500.0, servo_left_pwm = 1500.0, servo_right_pwm = 1500.0;
  double motor_neutral_pwm = 1500.0, motor_min_pwm = 1500.0, motor_max_pwm = 1500.0;
  double forward_pwm_per_mps = 160.0, reverse_pwm_per_mps = 160.0;
  double max_speed_mps = 0.5, speed_epsilon_mps = 0.01, command_timeout_s = 0.5;
  void validate() const {
    for (double v : {wheelbase_m, left_angle_deg, right_angle_deg, servo_center_pwm,
        servo_left_pwm, servo_right_pwm, motor_neutral_pwm, motor_min_pwm, motor_max_pwm,
        forward_pwm_per_mps, reverse_pwm_per_mps, max_speed_mps, speed_epsilon_mps, command_timeout_s})
      if (!std::isfinite(v)) throw std::invalid_argument("Non-finite calibration");
    if (wheelbase_m <= 0.0 || left_angle_deg <= 0.0 || left_angle_deg >= 90.0 ||
        right_angle_deg <= 0.0 || right_angle_deg >= 90.0 ||
        (servo_left_pwm-servo_center_pwm)*(servo_right_pwm-servo_center_pwm) >= 0.0 ||
        motor_min_pwm > motor_neutral_pwm || motor_max_pwm <= motor_neutral_pwm ||
        forward_pwm_per_mps <= 0.0 || reverse_pwm_per_mps <= 0.0 ||
        speed_epsilon_mps <= 0.0 || max_speed_mps <= speed_epsilon_mps ||
        command_timeout_s <= 0.0 || command_timeout_s > 2.0)
      throw std::invalid_argument("Incomplete/invalid steering or motor calibration");
    // Existing serial interface envelope, NOT mechanical travel limits.
    for (double v : {servo_center_pwm, servo_left_pwm, servo_right_pwm,
                    motor_neutral_pwm, motor_min_pwm, motor_max_pwm})
      if (v < 500.0 || v > 2500.0) throw std::invalid_argument("PWM outside [500,2500]");
  }
};
struct Command { uint16_t motor; uint16_t servo; bool valid; };
inline uint16_t pwm(double value, double lower, double upper) {
  return static_cast<uint16_t>(std::lround(std::clamp(value, lower, upper)));
}
inline Command neutral(const Calibration &c, bool valid = true) {
  return {pwm(c.motor_neutral_pwm,c.motor_min_pwm,c.motor_max_pwm),
          pwm(c.servo_center_pwm,500.0,2500.0),valid};
}
inline Command from_twist(const Calibration &c, double v, double omega) {
  if (!std::isfinite(v) || !std::isfinite(omega)) return neutral(c,false);
  if (std::abs(v) < c.speed_epsilon_mps) return neutral(c);
  // atan preserves reverse steering sign; atan2 would select the wrong quadrant.
  const double delta = std::atan(c.wheelbase_m * (omega/v)) * 180.0/kPi;
  const double bounded = std::clamp(delta,-c.right_angle_deg,c.left_angle_deg);
  const double servo = bounded >= 0.0
      ? c.servo_center_pwm + bounded/c.left_angle_deg*(c.servo_left_pwm-c.servo_center_pwm)
      : c.servo_center_pwm + (-bounded)/c.right_angle_deg*(c.servo_right_pwm-c.servo_center_pwm);
  if (c.speed_limit_enabled) v = std::clamp(v,-c.max_speed_mps,c.max_speed_mps);
  const double gain = v >= 0.0 ? c.forward_pwm_per_mps : c.reverse_pwm_per_mps;
  return {pwm(c.motor_neutral_pwm+v*gain,c.motor_min_pwm,c.motor_max_pwm),
          pwm(servo,std::min(c.servo_left_pwm,c.servo_right_pwm),
                    std::max(c.servo_left_pwm,c.servo_right_pwm)),true};
}
inline Command from_teleop(const Calibration &c, double motor, double degrees) {
  // Preserve legacy input: motor PWM, servo 0..180 with 90=center.
  if (!std::isfinite(motor) || !std::isfinite(degrees) || degrees < 0.0 || degrees > 180.0)
    return neutral(c,false);
  return {pwm(motor,c.motor_min_pwm,c.motor_max_pwm),
          pwm(c.servo_center_pwm+(90.0-degrees)*2000.0/180.0,std::min(c.servo_left_pwm,c.servo_right_pwm),
                                      std::max(c.servo_left_pwm,c.servo_right_pwm)),true};
}
class Watchdog {
 public:
  using Clock = std::chrono::steady_clock;
  explicit Watchdog(double timeout): timeout_(timeout) {}
  void received(Clock::time_point now) { last_=now; active_=true; }
  bool expired(Clock::time_point now) const {
    return active_ && std::chrono::duration<double>(now-last_).count() >= timeout_;
  }
  void stopped() { active_=false; }
 private:
  double timeout_;
  Clock::time_point last_{};
  bool active_=false;
};
}  // namespace racecar_control
