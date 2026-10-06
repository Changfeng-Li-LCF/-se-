#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace racecar_control {
// The injected writer MUST have a deadline. No ROS executor or callback is used.
class BlockingStop {
 public:
  enum Result { ok=0, io_error=1, io_timeout=2 };
  enum Fault { none=0, control_blocked=1, serial_blocked=2, serial_failed=3 };
  using Clock=std::chrono::steady_clock;
  using Writer=std::function<Result(uint16_t,uint16_t)>;
  using Reporter=std::function<void(Fault,bool)>;
  BlockingStop(Writer writer,Reporter reporter,uint16_t neutral,uint16_t center,
               bool brake,uint16_t brake_pwm,double brake_s,double timeout_s)
    : writer_(writer),reporter_(reporter),neutral_(neutral),center_(center),last_motor_(neutral),
      last_servo_(center),brake_(brake),brake_pwm_(brake_pwm),brake_s_(brake_s),timeout_s_(timeout_s) {}
  ~BlockingStop() { stop(); }
  void start() { running_=true;thread_=std::thread([this]{run();}); }
  void stop() {
    if(!running_.exchange(false))return;
    if(thread_.joinable())thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    writer_(neutral_,center_); // Never leave a brake pulse active on normal shutdown.
  }
  void heartbeat() {
    std::lock_guard<std::mutex> lock(mutex_);
    detect_locked();heartbeat_ns_=now_ns();armed_=true;
  }
  Fault fault() const { return fault_.load(); }
  bool send(uint16_t &motor,uint16_t &servo) {
    std::lock_guard<std::mutex> lock(mutex_);
    detect_locked();
    if (fault_!=none) effective_locked(motor,servo);
    const auto result=writer_(motor,servo);
    if (result==ok) { last_motor_=motor;last_servo_=servo;return true; }
    trip_locked(result==io_timeout ? serial_blocked:serial_failed);
    return false;
  }
  bool reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_==none) return true;
    if (!reported_ || reporting_ || !armed_ || age_s()>timeout_s_ || Clock::now()<brake_until_) return false;
    if (writer_(neutral_,center_)!=ok) return false;
    last_motor_=neutral_;last_servo_=center_;fault_=none;reported_=false;report_stop_failed_=false;
    return true;
  }
 private:
  static int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  }
  double age_s() const { return (now_ns()-heartbeat_ns_.load())*1e-9; }
  void trip_locked(Fault reason) {
    if (fault_!=none) return;
    held_servo_=center_;
    brake_until_=Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(brake_ && last_motor_>neutral_ ? brake_s_:0.0));
    fault_=reason;
  }
  void detect_locked() { if(armed_ && age_s()>timeout_s_)trip_locked(control_blocked); }
  void effective_locked(uint16_t &motor,uint16_t &servo) {
    motor=Clock::now()<brake_until_ ? brake_pwm_:neutral_;servo=held_servo_;
  }
  void run() {
    while(running_) {
      Fault report=none;bool delivered=false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        detect_locked();
        if(fault_!=none) {
          uint16_t motor=neutral_,servo=last_servo_;effective_locked(motor,servo);
          delivered=writer_(motor,servo)==ok;
          if(delivered){last_motor_=motor;last_servo_=servo;}
          // Report once at detection, and again if a later stop write fails.
          // Complete the configured brake pulse and attempt neutral BEFORE file/log I/O.
          if(Clock::now()>=brake_until_ && (!reported_ || (!delivered && !report_stop_failed_))) {
            report=fault_;reported_=true;report_stop_failed_=!delivered;reporting_=true;
          }
        }
      }
      // Stop is attempted before logging; never hold the serial mutex while logging.
      if(report!=none){reporter_(report,delivered);reporting_=false;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  Writer writer_;Reporter reporter_;
  const uint16_t neutral_,center_;
  uint16_t last_motor_,last_servo_,held_servo_=1500;
  const bool brake_;const uint16_t brake_pwm_;const double brake_s_,timeout_s_;
  std::atomic<int64_t> heartbeat_ns_{0};
  std::atomic<bool> running_{false},armed_{false};
  std::atomic<bool> reporting_{false};
  std::atomic<Fault> fault_{none};
  std::mutex mutex_;std::thread thread_;
  Clock::time_point brake_until_{};
  bool reported_=false,report_stop_failed_=false;
};
} // namespace racecar_control
