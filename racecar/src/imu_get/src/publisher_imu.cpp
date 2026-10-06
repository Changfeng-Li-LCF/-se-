#include <chrono>
#include <math.h>
#include "serial/serial.h"
#include <memory.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "transform.hpp"

serial::Serial ser;

using namespace std::chrono_literals;

class publisher_imu_node : public rclcpp::Node
{
public:
    std::string port;
    int baudrate;
    transform_imu imu_fetch;
    rclcpp::Time last_time_;
    int msg_count_;
    bool calibration_done_;
    
public:
    publisher_imu_node()
        : Node("publisher_imu_node"),
          last_time_(this->get_clock()->now()),
          msg_count_(0),
          calibration_done_(false)
    {
        // 100Hz = 10ms周期
        int output_hz = 100;
        auto timer_period = std::chrono::milliseconds(10);
        sample_period_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            timer_period).count();
        
        port = "/dev/imu";
        baudrate = 115200;
        
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser.setTimeout(to);
            ser.open();
            
            // 清空缓冲区
            ser.flushInput();
            ser.flushOutput();
            
            RCLCPP_INFO(this->get_logger(), "IMU port %s opened at %d baud", 
                       port.c_str(), baudrate);
        }
        catch (serial::IOException &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to open port %s: %s", 
                        port.c_str(), e.what());
            return;
        }
        catch (serial::SerialException &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Serial exception: %s", e.what());
            return;
        }

        if (!ser.isOpen())
        {
            RCLCPP_FATAL(this->get_logger(), "Serial port not open");
            return;
        }

        // 创建发布者，队列大小100确保不丢数据
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 100);
        pub_imu_offline = this->create_publisher<sensor_msgs::msg::Imu>("/IMU_data", 100);
        
        // 创建校准服务（可选）
        // calibrate_srv_ = this->create_service<std_srvs::srv::Empty>(
        //     "calibrate_imu", std::bind(&publisher_imu_node::calibrate_callback, this, _1, _2));

        // 创建定时器
        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&publisher_imu_node::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "IMU polling at %d Hz; checked-stream-v2 (publish every fresh gyro; host interval timestamp estimates, no hardware clock)", output_hz);
        RCLCPP_INFO(this->get_logger(), "IMU calibration applied");
        
        // 打印校准参数
        double acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
        imu_fetch.get_calibration(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
        RCLCPP_INFO(this->get_logger(), 
                   "Calibration offsets - Acc: (%.3f, %.3f, %.3f), Gyro: (%.5f, %.5f, %.5f)",
                   acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    }

    ~publisher_imu_node()
    {
        if (ser.isOpen())
        {
            ser.close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }

private:
    void timer_callback()
    {
        int available = ser.available();
        
        if (available > 0)
        {
            std::vector<unsigned char> read_buf(available);
            int bytes_read = ser.read(&read_buf[0], available);
            
            if (bytes_read > 0)
            {
                rclcpp::Time now = this->get_clock()->now();
                const double receipt_s = transform_imu::steady_seconds();
                
                // 转换为char类型
                std::vector<char> read_buf_char(bytes_read);
                for(int i = 0; i < bytes_read; i++)
                {
                    read_buf_char[i] = static_cast<char>(read_buf[i]);
                }
                
                // 解析数据（已包含校准）
                const auto before_errors = imu_fetch.stats.checksum_errors;
                const auto update = imu_fetch.FetchData(read_buf_char, bytes_read, receipt_s);
                if (imu_fetch.stats.checksum_errors != before_errors) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "IMU checksum failures: %llu; corrupted frames discarded",
                        static_cast<unsigned long long>(imu_fetch.stats.checksum_errors));
                }
                // Count every gyro for timing, including one rejected for stale
                // acceleration. Partial/acc/angle-only reads still publish none.
                const auto batch_time = sample_time_.estimate(update.samples.size(),
                    now.nanoseconds(), receipt_s, sample_period_ns_, transform_imu::freshness_s);
                if (batch_time.ros_time_rollback) {
                    RCLCPP_WARN(this->get_logger(),
                        "ROS clock moved backward; IMU timestamp anchor reset (timestamps cannot continue monotonically across the reset)");
                }
                if (!update.publishable) {
                    if (update.gyro_frames != 0) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "IMU gyro received without acceleration newer than 100 ms; skipping");
                    }
                    return;
                }
                // Each snapshot uses preceding acceleration/angle fields. The
                // host interval is divided by gyro count; no gyro is coalesced.
                for (std::size_t i = 0; i < update.samples.size(); ++i) {
                    const auto& sample = update.samples[i];
                    if (!sample.acceleration_valid) continue;
                    publish_imu_data(rclcpp::Time(batch_time.stamps_ns[i], now.get_clock_type()), sample);
                }
                
                // 调试输出（每100次输出一次）
                msg_count_ += static_cast<int>(std::count_if(update.samples.begin(), update.samples.end(),
                    [](const transform_imu::Sample& sample) { return sample.acceleration_valid; }));
                if (msg_count_ >= 100)
                {
                    double rate = msg_count_ / ((now - last_time_).seconds());
                    RCLCPP_DEBUG(this->get_logger(), 
                        "IMU Rate: %.1f Hz", rate);
                    
                    // 输出校准后的数据示例
                    RCLCPP_DEBUG(this->get_logger(),
                        "Calibrated Data - Acc: (%.3f, %.3f, %.3f) m/s², "
                        "Gyro: (%.5f, %.5f, %.5f) rad/s",
                        imu_fetch.acc.x, imu_fetch.acc.y, imu_fetch.acc.z,
                        imu_fetch.gyro.x, imu_fetch.gyro.y, imu_fetch.gyro.z);
                    
                    last_time_ = now;
                    msg_count_ = 0;
                }
            }
        }
    }
    
    void publish_imu_data(const rclcpp::Time& now, const transform_imu::Sample& sample)
    {
        sensor_msgs::msg::Imu imu_data;
        sensor_msgs::msg::Imu imu_offline_data;
        
        // ============== imu_data (/imu_data) ==============
        imu_data.header.stamp = now;
        imu_data.header.frame_id = "imu_link";
        
        // 加速度（已在校准中处理）
        imu_data.linear_acceleration.x = sample.acc.x;
        imu_data.linear_acceleration.y = sample.acc.y;
        imu_data.linear_acceleration.z = sample.acc.z;
        
        // 角速度（已在校准中处理）
        imu_data.angular_velocity.x = sample.gyro.x;
        imu_data.angular_velocity.y = sample.gyro.y;
        imu_data.angular_velocity.z = sample.gyro.z;
        
        // 四元数
        tf2::Quaternion quaternion;
        if (sample.orientation_valid) {
            quaternion.setRPY(sample.angle.r, sample.angle.p, sample.angle.y);
        } else {
            quaternion.setRPY(0.0, 0.0, 0.0);
        }
        quaternion.normalize();
        
        imu_data.orientation.x = quaternion.x();
        imu_data.orientation.y = quaternion.y();
        imu_data.orientation.z = quaternion.z();
        imu_data.orientation.w = quaternion.w();
        
        // 设置协方差矩阵
        set_covariance_matrix(imu_data);
        if (!sample.orientation_valid) imu_data.orientation_covariance[0] = -1.0;
        
        // ============== imu_offline_data (/IMU_data) ==============
        imu_offline_data.header.stamp = now;
        imu_offline_data.header.frame_id = "IMU_link";  // Cartographer使用
        
        // 复制校准后的数据
        imu_offline_data.linear_acceleration = imu_data.linear_acceleration;
        imu_offline_data.angular_velocity = imu_data.angular_velocity;
        imu_offline_data.orientation = imu_data.orientation;
        
        // 复制协方差矩阵
        for (int i = 0; i < 9; i++) {
            imu_offline_data.linear_acceleration_covariance[i] = 
                imu_data.linear_acceleration_covariance[i];
            imu_offline_data.angular_velocity_covariance[i] = 
                imu_data.angular_velocity_covariance[i];
            imu_offline_data.orientation_covariance[i] = 
                imu_data.orientation_covariance[i];
        }
        
        // 发布消息
        pub_imu->publish(imu_data);
        pub_imu_offline->publish(imu_offline_data);
    }
    
    void set_covariance_matrix(sensor_msgs::msg::Imu& imu_msg)
    {
        // 对角线协方差矩阵
        // 加速度协方差 (m/s²)²
        imu_msg.linear_acceleration_covariance[0] = 0.04;  // x轴
        imu_msg.linear_acceleration_covariance[4] = 0.04;  // y轴
        imu_msg.linear_acceleration_covariance[8] = 0.04;  // z轴
        
        // 角速度协方差 (rad/s)²
        imu_msg.angular_velocity_covariance[0] = 0.02;  // x轴
        imu_msg.angular_velocity_covariance[4] = 0.02;  // y轴
        imu_msg.angular_velocity_covariance[8] = 0.02;  // z轴
        
        // 方向协方差
        imu_msg.orientation_covariance[0] = 0.05;  // Roll
        imu_msg.orientation_covariance[4] = 0.05;  // Pitch
        imu_msg.orientation_covariance[8] = 0.05;  // Yaw
        
        // 非对角线元素设为0
        for (int i = 0; i < 9; i++) {
            if (i != 0 && i != 4 && i != 8) {
                imu_msg.linear_acceleration_covariance[i] = 0;
                imu_msg.angular_velocity_covariance[i] = 0;
                imu_msg.orientation_covariance[i] = 0;
            }
        }
    }
    
    ImuHostTime sample_time_;
    int64_t sample_period_ns_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_offline;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibrate_srv_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    printf("=========================================\n");
    printf("Starting IMU Publisher Node with Calibration\n");
    printf("=========================================\n");
    printf("IMU Port: /dev/imu\n");
    printf("Baudrate: 115200\n");
    printf("Frequency: 100Hz\n");
    printf("Calibration Applied\n");
    printf("=========================================\n");
    
    auto node = std::make_shared<publisher_imu_node>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        printf("IMU node exception: %s\n", e.what());
    }
    
    printf("\nIMU node shutdown\n");
    rclcpp::shutdown();
    return 0;
}
