#include "control_math.hpp"
#include "closed_loop.hpp"
#include "imu_feedback.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "racecar_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#ifndef RACECAR_CMD_TOPIC
#define RACECAR_CMD_TOPIC "/car_cmd_vel"
#endif
class RacecarDriver : public rclcpp::Node {
 public:
  RacecarDriver(): Node("racecar_driver"), watchdog_(0.5) {
    auto port=declare_parameter<std::string>("serial_port","/dev/car");
    const auto baud=declare_parameter<int>("baud_rate",38400);
    dry_run_=declare_parameter<bool>("dry_run",false);
    const bool confirmed=declare_parameter<bool>("calibration_confirmed",false);
    control_frequency_hz_=declare_parameter<double>("control_frequency_hz",20.0);
    if (!std::isfinite(control_frequency_hz_) || control_frequency_hz_<1.0 || control_frequency_hz_>200.0)
      throw std::invalid_argument("control_frequency_hz must be finite and within 1..200 Hz");
#define READ_CAL(field) c_.field=declare_parameter<double>(#field,c_.field)
    READ_CAL(wheelbase_m); READ_CAL(left_angle_deg); READ_CAL(right_angle_deg);
    READ_CAL(servo_center_pwm); READ_CAL(servo_left_pwm); READ_CAL(servo_right_pwm);
    READ_CAL(motor_neutral_pwm); READ_CAL(motor_min_pwm); READ_CAL(motor_max_pwm);
    READ_CAL(forward_pwm_per_mps); READ_CAL(reverse_pwm_per_mps);
    READ_CAL(max_speed_mps); READ_CAL(speed_epsilon_mps); READ_CAL(command_timeout_s);
#undef READ_CAL
    c_.speed_limit_enabled=declare_parameter<bool>("speed_limit_enabled",true);
    c_.validate();
    loop_settings_.speed_feedback_enabled=declare_parameter<bool>("speed_feedback_enabled",false);
    loop_settings_.yaw_rate_feedback_enabled=declare_parameter<bool>("yaw_rate_feedback_enabled",false);
    loop_settings_.speed_feedback_confidence_enabled=declare_parameter<bool>("speed_feedback_confidence_enabled",true);
#define READ_LOOP(field) loop_settings_.field=declare_parameter<double>(#field,loop_settings_.field)
    READ_LOOP(motor_kp); READ_LOOP(motor_ki);
    READ_LOOP(speed_setpoint_mps);
    READ_LOOP(motor_integral_limit_pwm); READ_LOOP(motor_correction_limit_pwm);
    READ_LOOP(motor_pwm_rise_rate_per_s); READ_LOOP(motor_pwm_fall_rate_per_s);
    READ_LOOP(motor_startup_pwm_rise_rate_per_s);
    READ_LOOP(motor_startup_pwm); READ_LOOP(motor_startup_timeout_s);
    READ_LOOP(motor_startup_motion_mps); READ_LOOP(motor_startup_confirm_s);
    READ_LOOP(yaw_kp_deg_per_radps); READ_LOOP(yaw_ki_deg_per_rad);
    READ_LOOP(yaw_integral_limit_deg); READ_LOOP(yaw_correction_limit_deg);
    READ_LOOP(yaw_feedback_min_speed_mps); READ_LOOP(feedback_timeout_s);
    READ_LOOP(feedback_filter_tau_s); READ_LOOP(feedback_max_speed_mps);
    READ_LOOP(feedback_max_yaw_rate_radps);
    READ_LOOP(feedback_lateral_noise_mps); READ_LOOP(feedback_lateral_ratio);
    READ_LOOP(feedback_turn_lever_arm_m);
    READ_LOOP(feedback_confidence_recovery_s);
#undef READ_LOOP
    loop_settings_.validate();
    feedback_topics_=declare_parameter<std::vector<std::string>>("feedback_odom_topics",{"/odom","/odom_rf2o"});
    feedback_base_frame_=declare_parameter<std::string>("feedback_base_frame","base_footprint");
    feedback_covariance_quality_enabled_=declare_parameter<bool>("feedback_covariance_quality_enabled",false);
    feedback_max_velocity_variance_=declare_parameter<double>("feedback_max_velocity_variance_mps2",0.25);
    if (!std::isfinite(feedback_max_velocity_variance_) || feedback_max_velocity_variance_<=0)
      throw std::invalid_argument("Velocity variance limit must be positive and finite");
    if (feedback_topics_.empty() || feedback_base_frame_.empty())
      throw std::invalid_argument("Velocity feedback requires topics and a base frame");
    feedback_.resize(feedback_topics_.size());
    imu_yaw_enabled_=declare_parameter<bool>("imu_yaw_feedback_enabled",false);
    imu_topic_=declare_parameter<std::string>("imu_feedback_topic","/IMU_data");
    imu_settings_.frame=declare_parameter<std::string>("imu_feedback_frame","IMU_link");
    const auto axis=declare_parameter<std::vector<double>>("imu_yaw_axis",{0.0,0.0,1.0});
    if (axis.size()!=3 || imu_topic_.empty()) throw std::invalid_argument("IMU requires topic and 3-element yaw axis");
    std::copy(axis.begin(),axis.end(),imu_settings_.axis.begin());
    imu_settings_.bias_radps=declare_parameter<double>("imu_yaw_bias_radps",0.0);
    imu_settings_.timeout_s=declare_parameter<double>("imu_feedback_timeout_s",0.15);
    imu_settings_.filter_tau_s=declare_parameter<double>("imu_filter_tau_s",0.02);
    imu_settings_.max_radps=declare_parameter<double>("imu_max_yaw_rate_radps",6.0);
    imu_settings_.validate();
    if (imu_yaw_enabled_ && !loop_settings_.yaw_rate_feedback_enabled)
      throw std::invalid_argument("IMU feedback requires yaw_rate_feedback_enabled");
    reverse_brake_enabled_=declare_parameter<bool>("reverse_brake_enabled",false);
    brake_pwm_=declare_parameter<double>("brake_pwm",c_.motor_neutral_pwm);
    brake_duration_s_=declare_parameter<double>("brake_duration_s",0.0);
    brake_min_duration_s_=declare_parameter<double>("brake_min_duration_s",0.0);
    brake_min_duration_speed_enabled_=declare_parameter<bool>("brake_min_duration_speed_enabled",false);
    brake_min_speed_reference_mps_=declare_parameter<double>("brake_min_speed_reference_mps",1.5);
    brake_min_duration_reference_s_=declare_parameter<double>("brake_min_duration_reference_s",1.2);
    brake_min_duration_speed_gain_s_per_mps_=declare_parameter<double>("brake_min_duration_speed_gain_s_per_mps",3.0);
    if (!std::isfinite(brake_min_speed_reference_mps_) || brake_min_speed_reference_mps_<=0.0 ||
        !std::isfinite(brake_min_duration_reference_s_) || brake_min_duration_reference_s_<0.0 ||
        !std::isfinite(brake_min_duration_speed_gain_s_per_mps_) || brake_min_duration_speed_gain_s_per_mps_<0.0)
      throw std::invalid_argument("Speed-based brake minimum requires finite positive reference speed and nonnegative duration/gain");
    if (!std::isfinite(brake_min_duration_s_) || brake_min_duration_s_<0.0 ||
        brake_min_duration_s_>brake_duration_s_)
      throw std::invalid_argument("Brake minimum duration must be within [0, brake_duration_s]");
    brake_stop_speed_mps_=declare_parameter<double>("brake_stop_speed_mps",0.03);
    brake_stop_confirm_s_=declare_parameter<double>("brake_stop_confirm_s",0.15);
    if (!std::isfinite(brake_stop_speed_mps_) || brake_stop_speed_mps_<=0 ||
        !std::isfinite(brake_stop_confirm_s_) || brake_stop_confirm_s_<=0)
      throw std::invalid_argument("Brake stop confirmation requires positive speed and duration");
    if (!std::isfinite(brake_pwm_) || !std::isfinite(brake_duration_s_) ||
        (reverse_brake_enabled_ && (brake_pwm_<500.0 || brake_pwm_>=c_.motor_neutral_pwm ||
                                   brake_duration_s_<=0.0 || brake_duration_s_>2.0)))
      throw std::invalid_argument("Enabled reverse brake requires PWM below neutral and a pulse in (0,2.0] seconds");
    const char *user_home=std::getenv("HOME");
    emergency_state_file_=declare_parameter<std::string>("emergency_stop_state_file",
        std::string(user_home ? user_home : "/tmp")+"/.local/state/racecar/emergency_stop");
    if (emergency_state_file_.empty())
      throw std::invalid_argument("Emergency-stop state file cannot be empty");
    std::error_code state_error;
    emergency_stopped_=std::filesystem::exists(emergency_state_file_,state_error) || bool(state_error);
    last_servo_pwm_=racecar_control::neutral(c_).servo;
    emergency_servo_pwm_=last_servo_pwm_;
    if (!dry_run_ && !confirmed)
      throw std::invalid_argument("Hardware calibration must be confirmed before opening serial port");
    watchdog_=racecar_control::Watchdog(c_.command_timeout_s);
    // Create ROS resources before opening the serial port.
    sub_=create_subscription<geometry_msgs::msg::Twist>(
        RACECAR_CMD_TOPIC,rclcpp::QoS(1),[this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          if (loop_settings_.enabled()) accept_target(racecar_control::twist_target(c_,m->linear.x,m->angular.z),0);
          else accept(racecar_control::from_twist(c_,m->linear.x,m->angular.z));
        });
    teleop_=create_subscription<geometry_msgs::msg::Twist>(
        "/teleop_cmd_vel",rclcpp::QoS(1),[this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          if (loop_settings_.enabled()) accept_target(racecar_control::teleop_target(c_,m->linear.x,m->angular.z),1);
          else accept(racecar_control::from_teleop(c_,m->linear.x,m->angular.z));
        });
    loop_state_pub_=create_publisher<std_msgs::msg::String>("~/closed_loop_state",rclcpp::QoS(5));
    stop_event_pub_=create_publisher<std_msgs::msg::String>(
        "~/stop_event",rclcpp::QoS(50).reliable().transient_local());
    stop_instance_=std::to_string(get_clock()->now().nanoseconds());

    if (loop_settings_.enabled()) {
      for (size_t i=0;i<feedback_topics_.size();++i)
        feedback_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
            feedback_topics_[i],rclcpp::QoS(1).best_effort(),[this,i](nav_msgs::msg::Odometry::ConstSharedPtr m) {
              if (m->child_frame_id!=feedback_base_frame_) {
                RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Ignoring velocity feedback in unexpected child frame");
                return;
              }
              const double stamp=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;
              const double ros_now=get_clock()->now().seconds(),steady_now=steady_seconds();
              const double quality_yaw=imu_yaw_enabled_ && imu_feedback_.fresh(imu_settings_,ros_now,steady_now)
                  ? imu_feedback_.sample.w : m->twist.twist.angular.z;
              const double variance=m->twist.covariance[0];
              const bool credible=!feedback_covariance_quality_enabled_ ||
                  (std::isfinite(variance) && variance>=0.0 && variance<=feedback_max_velocity_variance_);
              feedback_[i].push(loop_settings_,stamp,ros_now,steady_now,
                                m->twist.twist.linear.x,imu_yaw_enabled_ ? 0.0 : m->twist.twist.angular.z,
                                m->twist.twist.linear.y,quality_yaw,credible);
            }));
    }
    if (imu_yaw_enabled_) {
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(imu_topic_,rclcpp::QoS(1).best_effort(),
          [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
            const double stamp=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;
            if (!imu_feedback_.push(imu_settings_,m->header.frame_id,stamp,get_clock()->now().seconds(),
                steady_seconds(),{m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z},
                m->angular_velocity_covariance[0]))
              RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
                  "IMU yaw sample rejected (frame, timestamp, validity or range); waiting for valid feedback");
          });
    }
    boundary_guard_enabled_=declare_parameter<bool>("map_boundary_guard_enabled",false);
    boundary_guard_timeout_s_=declare_parameter<double>("map_boundary_guard_timeout_s",0.5);
    if (!std::isfinite(boundary_guard_timeout_s_) || boundary_guard_timeout_s_<=0)
      throw std::invalid_argument("Invalid map boundary guard timeout");
    boundary_guard_sub_=create_subscription<std_msgs::msg::Bool>(
      "/racecar/map_boundary_clear",rclcpp::QoS(1).reliable(),[this](std_msgs::msg::Bool::ConstSharedPtr m) {
        boundary_guard_seen_=true;boundary_guard_clear_=m->data;
        boundary_guard_received_=racecar_control::Watchdog::Clock::now();
        if (boundary_guard_enabled_ && !boundary_guard_clear_ && !emergency_stopped_ &&
            (control_active_ || last_motor_pwm_>racecar_control::neutral(c_).motor))
          boundary_stop();
      });
    emergency_state_pub_=create_publisher<std_msgs::msg::Bool>(
        "~/emergency_stop_active",rclcpp::QoS(1).reliable().transient_local());
    emergency_sub_=create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop",rclcpp::QoS(10).reliable(),[this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data) latch_emergency_stop("EXTERNAL_EMERGENCY_TOPIC");  // false must never release a stop.
        });
    emergency_service_=create_service<std_srvs::srv::Trigger>("~/emergency_stop",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success=latch_emergency_stop("EXPLICIT_EMERGENCY_SERVICE");
          response->message=response->success
            ? (braking_ ? "Reverse brake pulse started; motor will return to neutral and remain latched."
                        : "Emergency stop latched at neutral; no reverse pulse was issued.")
            : "Stop is latched in memory, but serial write or persistence failed; check hardware and logs.";
        });
    normal_stop_service_=create_service<std_srvs::srv::Trigger>("~/normal_stop",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          adopt_blocking_stop();
          // Routine cleanup/traffic stops merge with ordinary braking and never
          // reset its pulse deadline or create an emergency latch.
          if (!emergency_stopped_ && !normal_braking_) {
            pending_zero_before_=loop_state_json(steady_seconds(),get_clock()->now().seconds());
            accept(racecar_control::neutral(c_));
          }
          response->success=normal_braking_ || emergency_stopped_ ||
              last_motor_pwm_==racecar_control::neutral(c_).motor;
          response->message=emergency_stopped_ ? "Existing emergency stop remains latched." :
              (normal_braking_ ? "Normal braking continues; wait for completion before a fresh start."
                               : "Normal stop complete; a fresh motion command may start the car.");
        });
    reset_service_=create_service<std_srvs::srv::Trigger>("~/reset_emergency_stop",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          adopt_blocking_stop();
          if (!emergency_stopped_) {
            response->success=true;response->message="Emergency stop is already released.";return;
          }
          const auto now=racecar_control::Watchdog::Clock::now();
          if (braking_ && now<brake_deadline_) {
            response->success=false;response->message="Brake pulse is still active; reset refused.";return;
          }
          if (have_input_ && std::chrono::duration<double>(now-last_input_).count()<c_.command_timeout_s) {
            response->success=false;
            response->message="Stop command publishers first; reset requires a command-free timeout interval.";
            return;
          }
          if (!write_command(racecar_control::neutral(c_))) {
            response->success=false;response->message="Neutral write failed; emergency stop remains latched.";return;
          }
          if (blocking_stop_reset()!=0) {
            response->success=false;
            response->message="Blocking-stop protection remains latched: wait for a healthy control loop, completed brake pulse and successful neutral write.";
            return;
          }
          std::error_code error;
          std::filesystem::remove(emergency_state_file_,error);
          if (error) {
            response->success=false;response->message="Cannot clear persistent stop file; stop remains latched.";return;
          }
          emergency_stopped_=false;braking_=false;watchdog_.stopped();reset_control();publish_emergency_state();
          response->success=true;response->message="Emergency stop released. A new command is required.";
        });
    timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0/control_frequency_hz_)),[this]() { control_tick(); });
    if (!dry_run_ && art_racecar_init(baud,port.data()) != 0)
      throw std::runtime_error("Cannot initialize car serial port");
    ready_=true;
    std::filesystem::create_directories(std::filesystem::path(emergency_state_file_).parent_path());
    // The independent watchdog cannot observe fresh speed while the executor is
    // blocked. Cut drive to neutral; never infer braking permission from PWM.
    blocking_stop_start(static_cast<uint16_t>(c_.motor_neutral_pwm),last_servo_pwm_,
        false,static_cast<uint16_t>(brake_pwm_),brake_duration_s_,
        std::max(0.2,3.0/control_frequency_hz_),dry_run_,emergency_state_file_.c_str());
    write_command(racecar_control::neutral(c_));
    publish_emergency_state();
    if (emergency_stopped_)
      RCLCPP_WARN(get_logger(),"Persistent emergency stop is active; explicit reset is required");
    RCLCPP_INFO(get_logger(),"Calibrated Ackermann driver; dry_run=%s; topic=%s",
                dry_run_ ? "true" : "false",RACECAR_CMD_TOPIC);
    RCLCPP_INFO(get_logger(),"IMU yaw source: %s; topic=%s frame=%s; gyro-loop-v1",
                imu_yaw_enabled_ ? "direct_gyro":"disabled",imu_topic_.c_str(),imu_settings_.frame.c_str());
    RCLCPP_INFO(get_logger(),"Vehicle feedback: speed=%s yaw_rate=%s; %.1f Hz; body velocity, not shaft feedback",
                loop_settings_.speed_feedback_enabled ? "incremental_PI" : "off",
                loop_settings_.yaw_rate_feedback_enabled ? "PI" : "off",control_frequency_hz_);
    RCLCPP_INFO(get_logger(),"normal-stop-cleanup-v1: routine service preserves braking without latching; true emergencies preserve an active pulse");
    RCLCPP_INFO(get_logger(),"normal-brake-v3: ordinary stop applies brake PWM immediately; all stops center steering");
    RCLCPP_INFO(get_logger(),"reverse-brake-v3: fresh credible positive raw and filtered speed required throughout pulse; watchdog faults use neutral");
    RCLCPP_INFO(get_logger(),"blocking-stop-v1: independent control watchdog %.3f s; serial write deadline 0.020 s",
                std::max(0.2,3.0/control_frequency_hz_));
  }
  ~RacecarDriver() override {
    if (ready_) write_command(racecar_control::neutral(c_));
    blocking_stop_shutdown();
  }
 private:
  static double steady_seconds() {
    return std::chrono::duration<double>(racecar_control::Watchdog::Clock::now().time_since_epoch()).count();
  }
  void reset_control() {
    control_active_=false;target_={};loops_.reset();feedback_source_=-1;imu_feedback_started_=false;
  }
  bool boundary_allows_motion() const {
    return !boundary_guard_enabled_ || (boundary_guard_seen_ && boundary_guard_clear_ &&
      std::chrono::duration<double>(racecar_control::Watchdog::Clock::now()-boundary_guard_received_).count()<=boundary_guard_timeout_s_);
  }
  void boundary_stop() {
    feedback_stop("MAP_BOUNDARY_GUARD: blocked or heartbeat missing");
  }
  void feedback_stop(const char *reason) {
    RCLCPP_ERROR(get_logger(),"Closed-loop stop: %s",reason);
    latch_emergency_stop(reason);
  }
  void adopt_blocking_stop() {
    if (blocking_stop_fault() && !emergency_stopped_) {
      RCLCPP_ERROR(get_logger(),"BLOCKING_STOP reason=%s; independent stop is latched",blocking_stop_reason());
      // Independent guard has already cut drive; never start a reverse pulse.
      last_motor_pwm_=static_cast<uint16_t>(c_.motor_neutral_pwm);
      braking_=false;latch_emergency_stop(blocking_stop_reason());
    }
  }
  int fresh_stop_feedback(double steady_now,double ros_now) const {
    if (stop_feedback_source_>=0 && feedback_[stop_feedback_source_].fresh(loop_settings_,ros_now,steady_now))
      return stop_feedback_source_;
    for (size_t i=0;i<feedback_.size();++i)
      if (feedback_[i].fresh(loop_settings_,ros_now,steady_now)) return static_cast<int>(i);
    return -1;
  }
  bool forward_motion_feedback(double steady_now,double ros_now) const {
    const int index=fresh_stop_feedback(steady_now,ros_now);
    if (index<0 || !feedback_[index].credible(loop_settings_)) return false;
    const auto &sample=feedback_[index];
    // Raw speed prevents the low-pass filter from extending reverse torque
    // through zero. A positive command/PWM is not evidence of forward motion.
    return std::isfinite(sample.raw_v) && std::isfinite(sample.v) &&
        sample.raw_v>brake_stop_speed_mps_ && sample.v>brake_stop_speed_mps_;
  }
  double normal_brake_minimum(double speed) const {
    if (!brake_min_duration_speed_enabled_ || !std::isfinite(speed) || speed<=0.0)
      return brake_min_duration_s_;
    return std::clamp(brake_min_duration_reference_s_+
        brake_min_duration_speed_gain_s_per_mps_*(speed-brake_min_speed_reference_mps_),
        brake_min_duration_s_,brake_duration_s_);
  }
  bool stop_confirmed(double steady_now,double ros_now) {
    stop_feedback_source_=fresh_stop_feedback(steady_now,ros_now);
    return stop_confirmation_.update(stop_feedback_source_>=0 ? &feedback_[stop_feedback_source_] : nullptr,
        stop_feedback_source_,loop_settings_,ros_now,steady_now,brake_stop_speed_mps_,brake_stop_confirm_s_);
  }
  void accept_target(racecar_control::Target target,int source) {
    adopt_blocking_stop();
    target=racecar_control::shared_speed_target(c_,loop_settings_,target,source==0);
    last_input_=racecar_control::Watchdog::Clock::now();have_input_=true;
    if (emergency_stopped_) {
      write_command(emergency_command());
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Motion command ignored: emergency stop is latched");
      return;
    }

    if (target.valid && std::abs(target.v)>=c_.speed_epsilon_mps && !boundary_allows_motion()) {
      boundary_stop();return;
    }
    if (normal_braking_) return; // Complete the pulse; require a fresh command afterwards.
    if (!target.valid || std::abs(target.v)<c_.speed_epsilon_mps) {
      pending_zero_before_=loop_state_json(steady_seconds(),get_clock()->now().seconds());
      // Explicit stop remains distinct from PI temporarily coasting at neutral.
      accept({racecar_control::neutral(c_).motor,
              racecar_control::neutral(c_).servo,
              target.valid});
      return;
    }
    if (!control_active_ || source!=command_source_ || target.v*target_.v<0) loops_.reset();
    target_=target;command_source_=source;control_active_=true;
    watchdog_.received(last_input_); // Control ticks must not renew the command watchdog.
  }
  std::string loop_state_json(double now_s,double ros_now) const {
    const int state_source=feedback_source_>=0 ? feedback_source_ :
        ((normal_braking_ || braking_) ? stop_feedback_source_ : -1);
    const bool selected=state_source>=0;
    const auto sample=selected ? feedback_[state_source] : racecar_control::FeedbackSample{};
    const bool imu_fresh=imu_feedback_.fresh(imu_settings_,ros_now,now_s);
    const double measured_w=imu_yaw_enabled_ ? imu_feedback_.sample.w : sample.w;
    std::ostringstream json;
    json << "{\"stamp\":" << std::fixed << ros_now
         << ",\"last_stop_id\":" << std::quoted(last_stop_id_)
         << ",\"last_stop_reason\":" << std::quoted(last_stop_reason_)
         << ",\"command_source\":" << command_source_
         << ",\"command_age_s\":" << (have_input_ ? now_s-std::chrono::duration<double>(last_input_.time_since_epoch()).count() : -1.0)
         << ",\"velocity_feedback_age_s\":" << (selected ? ros_now-sample.stamp : -1.0)
         << ",\"imu_feedback_age_s\":" << (imu_feedback_.sample.stamp>0 ? ros_now-imu_feedback_.sample.stamp : -1.0)
         << ",\"boundary_guard_clear\":" << (boundary_guard_clear_ ? "true":"false")
         << ",\"boundary_guard_age_s\":" << (boundary_guard_seen_ ? now_s-std::chrono::duration<double>(boundary_guard_received_.time_since_epoch()).count() : -1.0)
         << ",\"blocking_stop_reason\":\"" << blocking_stop_reason() << "\""
         << ",\"speed_closed_loop\":" << (loop_settings_.speed_feedback_enabled ? "true":"false")
         << ",\"yaw_rate_closed_loop\":" << (loop_settings_.yaw_rate_feedback_enabled ? "true":"false")
         << ",\"normal_braking\":" << (normal_braking_ ? "true":"false")
         << ",\"normal_brake_trigger_speed_mps\":" << normal_brake_trigger_speed_mps_
         << ",\"normal_brake_min_duration_s\":" << normal_brake_min_duration_s_
         << ",\"reverse_brake_permitted\":" << (reverse_brake_enabled_ && !blocking_stop_fault() &&
             ((normal_braking_ && !normal_decelerating_ && racecar_control::Watchdog::Clock::now()<normal_brake_min_deadline_) ||
              forward_motion_feedback(now_s,ros_now)) ? "true":"false")
         << ",\"stop_phase\":" << std::quoted(normal_decelerating_ ? "decelerating" :
              ((normal_braking_ || braking_) ? "braking" : (emergency_stopped_ ? "latched" : "idle")))
         << ",\"active\":" << (control_active_ ? "true":"false")
         << ",\"emergency_stopped\":" << (emergency_stopped_ ? "true":"false")
         << ",\"feedback_fresh\":" << (selected && sample.fresh(loop_settings_,ros_now,now_s) && (!imu_yaw_enabled_ || imu_fresh) ? "true":"false")
         << ",\"feedback_source_index\":" << state_source
         << ",\"feedback_stamp\":" << sample.stamp
         << ",\"speed_feedback_confidence_enabled\":" << (loop_settings_.speed_feedback_confidence_enabled ? "true":"false")
         << ",\"speed_feedback_credible\":" << (selected && sample.credible(loop_settings_) ? "true":"false")
         << ",\"speed_feedback_confidence_reason\":" << std::quoted(sample.confidence_reason)
         << ",\"raw_measured_v\":" << sample.raw_v << ",\"measured_lateral_v\":" << sample.lateral_v
         << ",\"feedback_lateral_limit_mps\":" << sample.lateral_limit
         << ",\"motor_feedback_hold\":" << (loops_.motor_feedback_hold() ? "true":"false")
         << ",\"yaw_feedback_suspended_for_speed\":" << (control_active_ && loop_settings_.yaw_rate_feedback_enabled && !sample.credible(loop_settings_) ? "true":"false")
         << ",\"target_v\":" << target_.v << ",\"measured_v\":" << sample.v
         << ",\"target_w\":" << racecar_control::feedback_yaw_target(c_,loop_settings_,target_,sample.v)
         << ",\"reference_w\":" << target_.w << ",\"measured_w\":" << measured_w
         << ",\"yaw_feedback_source\":\"" << (imu_yaw_enabled_ ? "imu":"odom") << "\""
         << ",\"imu_feedback_fresh\":" << (imu_fresh ? "true":"false")
         << ",\"imu_feedback_stamp\":" << imu_feedback_.sample.stamp
         << ",\"imu_raw_w\":" << imu_feedback_.raw_w
         << ",\"imu_samples_accepted\":" << imu_feedback_.accepted
         << ",\"imu_samples_rejected\":" << imu_feedback_.rejected
         << ",\"base_steering_deg\":" << target_.steering_deg
         << ",\"yaw_correction_deg\":" << (control_active_ ? racecar_control::steering_degrees(c_,last_servo_pwm_)-target_.steering_deg : 0.0)
         << ",\"motor_integral_pwm\":" << loops_.motor_integral(loop_settings_)
         << ",\"motor_control_mode\":\"" << (loop_settings_.speed_feedback_enabled ? "incremental_pi":"open_loop") << "\""
         << ",\"motor_startup_active\":" << (loops_.motor_starting() ? "true":"false")
         << ",\"yaw_integral_deg\":" << loops_.yaw_integral()
         << ",\"last_written_motor_pwm\":" << last_motor_pwm_
         << ",\"last_written_servo_pwm\":" << last_servo_pwm_ << "}";
    return json.str();
  }
  void report_stop(const char *reason,const char *kind,const std::string &before) {
    const double stamp=get_clock()->now().seconds();
    last_stop_id_=stop_instance_+"-"+std::to_string(++stop_sequence_);
    last_stop_reason_=reason;
    std::ostringstream out;
    out << "{\"id\":" << std::quoted(last_stop_id_) << ",\"stamp\":" << std::fixed << stamp
        << ",\"kind\":" << std::quoted(kind) << ",\"reason\":" << std::quoted(reason)
        << ",\"before\":" << before << ",\"after\":" << loop_state_json(steady_seconds(),stamp) << "}";
    std_msgs::msg::String msg;msg.data=out.str();stop_event_pub_->publish(msg);
  }
  void publish_loop_state(double now_s,double ros_now) {
    if (now_s-last_state_publish_<0.05) return;
    last_state_publish_=now_s;
    std_msgs::msg::String message;message.data=loop_state_json(now_s,ros_now);loop_state_pub_->publish(message);
  }
  void control_tick() {
    blocking_stop_heartbeat();
    adopt_blocking_stop();
    const auto now=racecar_control::Watchdog::Clock::now();
    const double steady_now=steady_seconds(),ros_now=get_clock()->now().seconds();
    const double dt=last_tick_>0 ? steady_now-last_tick_ : 1.0/control_frequency_hz_;
    last_tick_=steady_now;
    if (last_ros_now_>0 && ros_now<last_ros_now_-0.05) {
      for (auto &sample:feedback_) sample.reset();
      imu_feedback_.reset();
      if (control_active_) feedback_stop("ROS clock moved backwards");
    }
    last_ros_now_=ros_now;
    if (emergency_stopped_) {
      if (braking_ && (now>=brake_deadline_ || !forward_motion_feedback(steady_now,ros_now) ||
                      stop_confirmed(steady_now,ros_now))) braking_=false;
      write_command(emergency_command());publish_loop_state(steady_now,ros_now);return;
    }
    if (normal_braking_) {
      // Once reverse output begins, honor the explicitly configured minimum pulse.
      // Before it begins, retain the forward-motion check; emergencies still take precedence.
      const bool stopped=!forward_motion_feedback(steady_now,ros_now) || stop_confirmed(steady_now,ros_now);
      if (normal_decelerating_ && !stopped) {
        const double elapsed=std::chrono::duration<double>(now-normal_stop_started_).count();
        const auto motor=static_cast<uint16_t>(std::lround(std::max(c_.motor_neutral_pwm,
            normal_stop_motor_pwm_-loop_settings_.motor_pwm_fall_rate_per_s*elapsed)));
        if (motor>racecar_control::neutral(c_).motor) {
          if (!write_command({motor,normal_stop_servo_pwm_,true}))
            feedback_stop("normal stop deceleration write failed");
          publish_loop_state(steady_now,ros_now);return;
        }
        normal_decelerating_=false;
        normal_brake_deadline_=now+std::chrono::duration_cast<racecar_control::Watchdog::Clock::duration>(
            std::chrono::duration<double>(brake_duration_s_));
        normal_brake_min_deadline_=now+std::chrono::duration_cast<racecar_control::Watchdog::Clock::duration>(
            std::chrono::duration<double>(normal_brake_min_duration_s_));
        stop_confirmation_.reset(steady_now);
      }
      if ((stopped && (normal_decelerating_ || now>=normal_brake_min_deadline_)) || now>=normal_brake_deadline_) {
        normal_braking_=false;normal_decelerating_=false;
        if (!write_command({racecar_control::neutral(c_).motor,normal_stop_servo_pwm_,true}))
          feedback_stop("normal brake completion write failed");
      } else if (!write_command({static_cast<uint16_t>(std::lround(brake_pwm_)),normal_brake_servo_pwm_,true})) {
        feedback_stop("normal brake pulse write failed");
      }
      publish_loop_state(steady_now,ros_now);return;
    }
    if ((control_active_ || last_motor_pwm_>racecar_control::neutral(c_).motor) && !boundary_allows_motion()) {
      boundary_stop();
      publish_loop_state(steady_now,ros_now);return;
    }
    if (watchdog_.expired(now)) {
      const auto before=loop_state_json(steady_now,ros_now);
      const bool was_active=control_active_;
      reset_control();
      if (reverse_brake_enabled_ && last_motor_pwm_>racecar_control::neutral(c_).motor)
        latch_emergency_stop("COMMAND_TIMEOUT",before);
      else if (was_active) {
        write_command(racecar_control::neutral(c_));watchdog_.stopped();
        report_stop("COMMAND_TIMEOUT","command_timeout",before);
      }
      else if (write_command(racecar_control::neutral(c_))) watchdog_.stopped();
      RCLCPP_WARN(get_logger(),"Command timeout: stopping");
      publish_loop_state(steady_now,ros_now);return;
    }
    if (!control_active_) { publish_loop_state(steady_now,ros_now);return; }
    if (dt<=0 || dt>0.2) {
      feedback_stop("CONTROL_LOOP_BLOCKED: control timer delayed beyond 0.2 seconds");publish_loop_state(steady_now,ros_now);return;
    }
    if (feedback_source_>=0 && !feedback_[feedback_source_].fresh(loop_settings_,ros_now,steady_now)) {
      feedback_stop("VELOCITY_FEEDBACK_STALL: feedback expired; possible source blocking/interruption/delay, cause unconfirmed; no open-loop fallback");
      publish_loop_state(steady_now,ros_now);return;
    }
    if (feedback_source_<0) {
      for (size_t i=0;i<feedback_.size();++i) {
        if (feedback_[i].fresh(loop_settings_,ros_now,steady_now)) { feedback_source_=static_cast<int>(i);break; }
      }
      if (feedback_source_<0) {
        loops_.reset();write_command(racecar_control::neutral(c_));
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Waiting for fresh velocity feedback; motor neutral");
        publish_loop_state(steady_now,ros_now);return;
      }
      RCLCPP_INFO(get_logger(),"Closed-loop feedback source: %s",feedback_topics_[feedback_source_].c_str());
    }
    if (imu_yaw_enabled_ && !imu_feedback_.fresh(imu_settings_,ros_now,steady_now)) {
      if (imu_feedback_started_) feedback_stop("IMU_FEEDBACK_STALL: feedback expired; possible source blocking/interruption/delay, cause unconfirmed; no odometry fallback");
      else {
        loops_.reset();write_command(racecar_control::neutral(c_));
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Waiting for fresh IMU yaw feedback; motor neutral");
      }
      publish_loop_state(steady_now,ros_now);return;
    }
    if (imu_yaw_enabled_) imu_feedback_started_=true;
    const auto &feedback=feedback_[feedback_source_];
    const auto output=loops_.update(c_,loop_settings_,target_,feedback.v,imu_yaw_enabled_ ? imu_feedback_.sample.w : feedback.w,dt,
                                   feedback.credible(loop_settings_));
    // A neutral correction is coasting, not a stop request: do not repeatedly reverse-brake.
    if (!output.valid || !write_command(output)) feedback_stop("invalid controller output or serial write failure");
    publish_loop_state(steady_now,ros_now);
  }
  racecar_control::Command emergency_command() {
    const bool pulse=braking_ && racecar_control::Watchdog::Clock::now()<brake_deadline_ &&
        forward_motion_feedback(steady_seconds(),get_clock()->now().seconds());
    return {pulse ? static_cast<uint16_t>(std::lround(brake_pwm_)) : racecar_control::neutral(c_).motor,
            emergency_servo_pwm_,true};
  }
  void publish_emergency_state() {
    std_msgs::msg::Bool message;message.data=emergency_stopped_;emergency_state_pub_->publish(message);
  }
  bool latch_emergency_stop(const char *reason="EXPLICIT_EMERGENCY_STOP",
                            const std::string &captured_before="") {
    const bool first=!emergency_stopped_;
    const auto before=first ? (captured_before.empty() ? loop_state_json(steady_seconds(),get_clock()->now().seconds()) : captured_before) : "";
    const bool preserve_normal_pulse=normal_braking_ && !normal_decelerating_;
    // A real emergency still latches, but must not cancel an existing reverse
    // brake pulse merely because the last PWM is now below neutral.
    normal_braking_=false;normal_decelerating_=false;
    if (!emergency_stopped_) {
      emergency_servo_pwm_=racecar_control::neutral(c_).servo;
      if (!preserve_normal_pulse) {
        stop_feedback_source_=feedback_source_;
        stop_confirmation_.reset(steady_seconds());
      }
      // A forward command cannot prove the car is
      // still rolling. Existing pulses also lose permission when feedback fails.
      braking_=reverse_brake_enabled_ && !blocking_stop_fault() &&
          forward_motion_feedback(steady_seconds(),get_clock()->now().seconds());
      brake_deadline_=preserve_normal_pulse ? normal_brake_deadline_ : racecar_control::Watchdog::Clock::now()+
        std::chrono::duration_cast<racecar_control::Watchdog::Clock::duration>(
            std::chrono::duration<double>(brake_duration_s_));
    }
    emergency_stopped_=true;watchdog_.stopped();reset_control();
    // Send stop output before filesystem operations; retries run on the configured control timer.
    const bool written=write_command(emergency_command());
    if (first) report_stop(reason,"emergency",before);
    bool saved=false;
    try {
      const std::filesystem::path file(emergency_state_file_);
      if (!file.parent_path().empty()) std::filesystem::create_directories(file.parent_path());
      std::ofstream output(file,std::ios::trunc);
      output << "Emergency stop latched. Release through reset_emergency_stop service.\n";
      output.close();saved=bool(output);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"Cannot persist emergency stop: %s",e.what());
    }
    publish_emergency_state();
    RCLCPP_WARN(get_logger(),"Emergency stop latched; reverse pulse=%s",braking_ ? "true" : "false");
    return written && saved;
  }
  void accept(racecar_control::Command cmd) {
    adopt_blocking_stop();
    last_input_=racecar_control::Watchdog::Clock::now();have_input_=true;
    if (emergency_stopped_) {
      write_command(emergency_command());
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Motion command ignored: emergency stop is latched");
      return;
    }
    if (cmd.valid && cmd.motor>racecar_control::neutral(c_).motor && !boundary_allows_motion()) {
      boundary_stop();return;
    }
    if (normal_braking_) return;
    if (!cmd.valid) { feedback_stop("invalid motion command");return; }
    if (cmd.motor==racecar_control::neutral(c_).motor) cmd.servo=racecar_control::neutral(c_).servo;
    if (reverse_brake_enabled_ && cmd.motor==racecar_control::neutral(c_).motor &&
        forward_motion_feedback(steady_seconds(),get_clock()->now().seconds())) {
      const auto before=pending_zero_before_.empty() ? loop_state_json(steady_seconds(),get_clock()->now().seconds()) : pending_zero_before_;
      pending_zero_before_.clear();
      // A normal zero command brakes once, without persisting an emergency stop.
      stop_feedback_source_=fresh_stop_feedback(steady_seconds(),get_clock()->now().seconds());
      normal_brake_trigger_speed_mps_=stop_feedback_source_>=0 ? feedback_[stop_feedback_source_].v : 0.0;
      normal_brake_min_duration_s_=normal_brake_minimum(normal_brake_trigger_speed_mps_);
      reset_control();watchdog_.stopped();normal_braking_=true;
      normal_brake_servo_pwm_=normal_stop_servo_pwm_=racecar_control::neutral(c_).servo;
      normal_stop_started_=racecar_control::Watchdog::Clock::now();
      normal_stop_motor_pwm_=last_motor_pwm_;
      normal_decelerating_=false; // Apply reverse brake on this command, without a forward PWM ramp.
      stop_confirmation_.reset(steady_seconds());
      normal_brake_deadline_=racecar_control::Watchdog::Clock::now()+
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(brake_duration_s_));
      normal_brake_min_deadline_=racecar_control::Watchdog::Clock::now()+
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(normal_brake_min_duration_s_));
      if (!write_command({static_cast<uint16_t>(std::lround(brake_pwm_)),
                          normal_brake_servo_pwm_,true}))
        feedback_stop("normal brake pulse write failed");
      else {
        RCLCPP_INFO(get_logger(),"Normal brake: trigger_speed=%.3f m/s minimum=%.3f s maximum=%.3f s PWM=%.0f",
                    normal_brake_trigger_speed_mps_,normal_brake_min_duration_s_,brake_duration_s_,brake_pwm_);
        report_stop("ZERO_SPEED_COMMAND","normal",before);
      }
      return;
    }
    if (cmd.motor==racecar_control::neutral(c_).motor && !pending_zero_before_.empty()) {
      const auto before=pending_zero_before_;pending_zero_before_.clear();
      if (before.find("\"active\":true")!=std::string::npos)
        report_stop("ZERO_SPEED_COMMAND","normal",before);
    }
    pending_zero_before_.clear();
    if (cmd.motor==racecar_control::neutral(c_).motor) reset_control();
    if (!cmd.valid)
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Invalid command rejected; stopping");
    write_command(cmd);
    watchdog_.received(racecar_control::Watchdog::Clock::now());
  }
  bool write_command(racecar_control::Command cmd) {
    if (send_cmd_guarded(&cmd.motor,&cmd.servo) != 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Serial command write failed");
      return false;
    }
    if(dry_run_)RCLCPP_INFO(get_logger(),"DRY_RUN motor=%u servo=%u",cmd.motor,cmd.servo);
    last_servo_pwm_=cmd.servo;
    last_motor_pwm_=cmd.motor;
    return true;
  }
  racecar_control::ImuYawSettings imu_settings_;
  racecar_control::ImuYawFeedback imu_feedback_;
  bool imu_yaw_enabled_=false,imu_feedback_started_=false;
  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  racecar_control::Calibration c_;
  racecar_control::LoopSettings loop_settings_;
  racecar_control::VehicleLoops loops_;
  racecar_control::Target target_;
  std::vector<racecar_control::FeedbackSample> feedback_;
  std::vector<std::string> feedback_topics_;
  std::string feedback_base_frame_;
  bool feedback_covariance_quality_enabled_=false;
  double feedback_max_velocity_variance_=0.25;
  int feedback_source_=-1,command_source_=0;
  bool control_active_=false;
  double last_tick_=0,last_ros_now_=0,last_state_publish_=0;
  double control_frequency_hz_=20.0;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> feedback_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loop_state_pub_;
  racecar_control::Watchdog watchdog_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_event_pub_;
  std::string stop_instance_,last_stop_id_,last_stop_reason_,pending_zero_before_;
  uint64_t stop_sequence_=0;
  bool boundary_guard_enabled_=false,boundary_guard_seen_=false,boundary_guard_clear_=false;
  double boundary_guard_timeout_s_=0.5;
  racecar_control::Watchdog::Clock::time_point boundary_guard_received_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boundary_guard_sub_;
  bool dry_run_=false,ready_=false;
  bool normal_braking_=false,normal_decelerating_=false;
  double normal_stop_motor_pwm_=1500;
  double normal_brake_trigger_speed_mps_=0.0,normal_brake_min_duration_s_=0.0;
  racecar_control::Watchdog::Clock::time_point normal_stop_started_{};
  racecar_control::Watchdog::Clock::time_point normal_brake_deadline_{};
  racecar_control::Watchdog::Clock::time_point normal_brake_min_deadline_{};
  uint16_t normal_brake_servo_pwm_=1500,normal_stop_servo_pwm_=1500;
  bool emergency_stopped_=false,have_input_=false;
  bool reverse_brake_enabled_=false,braking_=false;
  double brake_pwm_=1500.0,brake_duration_s_=0.0,brake_min_duration_s_=0.0;
  bool brake_min_duration_speed_enabled_=false;
  double brake_min_speed_reference_mps_=1.5,brake_min_duration_reference_s_=1.2;
  double brake_min_duration_speed_gain_s_per_mps_=3.0;
  double brake_stop_speed_mps_=0.03,brake_stop_confirm_s_=0.15;
  int stop_feedback_source_=-1;
  racecar_control::BrakeStopConfirmation stop_confirmation_;
  racecar_control::Watchdog::Clock::time_point brake_deadline_{};
  uint16_t last_servo_pwm_=1500,emergency_servo_pwm_=1500;
  uint16_t last_motor_pwm_=1500;
  std::string emergency_state_file_;
  racecar_control::Watchdog::Clock::time_point last_input_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_,teleop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_,reset_service_,normal_stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    auto node=std::make_shared<RacecarDriver>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("racecar_driver"),"%s",e.what());
    result=1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
