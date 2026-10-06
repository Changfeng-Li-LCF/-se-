#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <string>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <vector>
#include <cstdint>
#include <cstdio>

// Host receipt interpolation, NOT a hardware acquisition timestamp. The last
// sample is anchored at receipt; earlier samples retain their serial order.
class ImuHostTime
{
private:
    int64_t last_receipt_ns_ = 0;
    double last_receipt_s_ = -std::numeric_limits<double>::infinity();

public:
    struct Batch {
        std::vector<int64_t> stamps_ns;
        bool reanchored = false;
        bool ros_time_rollback = false;
    };

    Batch estimate(std::size_t count, int64_t receipt_ns, double receipt_s,
                   int64_t initial_spacing_ns, double reset_after_s)
    {
        Batch result;
        if (count == 0) return result;
        const bool had_previous = std::isfinite(last_receipt_s_);
        result.ros_time_rollback = had_previous && receipt_ns < last_receipt_ns_;
        result.reanchored = !had_previous ||
            receipt_ns <= last_receipt_ns_ || receipt_s < last_receipt_s_ ||
            receipt_s - last_receipt_s_ > reset_after_s;
        // After an outage/clock reset there is no continuous host interval.
        // Backdate using the existing polling period; never stamp after receipt.
        int64_t span_ns = result.reanchored ?
            static_cast<int64_t>(count) * initial_spacing_ns :
            receipt_ns - last_receipt_ns_;
        // A reset must not move the first sample behind the previous batch
        // while ROS time still advances, even if many gyros arrive together.
        if (had_previous && receipt_ns > last_receipt_ns_)
            span_ns = std::min(span_ns, receipt_ns - last_receipt_ns_);
        const int64_t start_ns = std::max<int64_t>(0, receipt_ns - span_ns);
        result.stamps_ns.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            result.stamps_ns.push_back(start_ns +
                (receipt_ns - start_ns) * static_cast<int64_t>(i + 1) /
                static_cast<int64_t>(count));
        }
        last_receipt_ns_ = receipt_ns;
        last_receipt_s_ = receipt_s;
        return result;
    }
};

class transform_imu
{
private:
    std::array<uint8_t, 11> pending_{};
    std::size_t pending_size_ = 0;
    double last_input_s_ = -std::numeric_limits<double>::infinity();
    double acc_time_s_ = -std::numeric_limits<double>::infinity();
    double angle_time_s_ = -std::numeric_limits<double>::infinity();

    void discard_front()
    {
        std::move(pending_.begin() + 1, pending_.begin() + pending_size_, pending_.begin());
        --pending_size_;
        ++stats.discarded_bytes;
    }

    // 最终校准参数（基于你的最新数据）
    struct Calibration {
        // 加速度偏移量（静止时：x=0, y=0, z=9.81）
        // 计算：offset = 原始值 - 目标值
        double acc_x_offset = 0.999;    // 目标：x=0, 当前：x≈0.082
        double acc_y_offset = 0.307;    // 目标：y=0, 当前：y≈-0.058
        double acc_z_offset = 0.014;    // 目标：z=9.81, 当前：z≈9.767
        
        // 陀螺仪偏移量（静止时：x=0, y=0, z=0）
        double gyro_x_offset = -0.00213; // 目标：x=0, 当前：x≈-0.00213
        double gyro_y_offset = 0.00106;  // 目标：y=0, 当前：y≈0.00106
        double gyro_z_offset = 0.0;      // 目标：z=0, 当前：z≈-4.7e-06
    } calib;

public:
    struct Acc
    {
        double x;
        double y;
        double z;
    } acc{0,0,0};
    struct Gyro
    {
        double x;
        double y;
        double z;
    } gyro{0,0,0};
    struct Angle
    {
        double r;
        double p;
        double y;
    } angle{0,0,0};

    struct Statistics {
        uint64_t valid_frames = 0;
        uint64_t gyro_frames = 0;
        uint64_t checksum_errors = 0;
        uint64_t discarded_bytes = 0;
        uint64_t partial_timeouts = 0;
    } stats;

    struct Sample {
        Acc acc;
        Gyro gyro;
        Angle angle;
        bool acceleration_valid;
        bool orientation_valid;
    };

    struct Update {
        unsigned acc_frames = 0;
        unsigned gyro_frames = 0;
        unsigned angle_frames = 0;
        bool publishable = false;
        bool orientation_valid = false;
        std::vector<Sample> samples;
    };

    // Only host reception freshness is known; this is not a sensor timestamp.
    static constexpr double freshness_s = 0.1;
    static double steady_seconds()
    {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::size_t pending_bytes() const { return pending_size_; }

    Update FetchData(const std::vector<char>& data, int length,
                     double receipt_s = steady_seconds())
    {
        Update update;
        const auto count = std::min(data.size(),
            static_cast<std::size_t>(std::max(0, length)));
        if (count == 0) return update;
        if (receipt_s < last_input_s_ || receipt_s - last_input_s_ > freshness_s) {
            if (pending_size_ != 0) {
                stats.discarded_bytes += pending_size_;
                ++stats.partial_timeouts;
                pending_size_ = 0;
            }
            // Do not reuse fields across a serial outage or clock reset.
            acc_time_s_ = angle_time_s_ = -std::numeric_limits<double>::infinity();
        }
        last_input_s_ = receipt_s;

        for (std::size_t i = 0; i < count; ++i) {
            pending_[pending_size_++] = static_cast<uint8_t>(data[i]);
            while (pending_size_ && pending_[0] != 0x55) discard_front();
            if (pending_size_ < 11) continue;
            uint8_t checksum = 0;
            for (std::size_t j = 0; j < 10; ++j) checksum += pending_[j];
            if (checksum != pending_[10]) {
                ++stats.checksum_errors;
                discard_front();
                while (pending_size_ && pending_[0] != 0x55) discard_front();
                continue;
            }
            ++stats.valid_frames;
            const auto read_s16 = [&](std::size_t offset) -> int32_t {
                const uint32_t raw = pending_[offset] |
                    (static_cast<uint32_t>(pending_[offset + 1]) << 8);
                return raw >= 32768 ? static_cast<int32_t>(raw) - 65536 : raw;
            };
            switch (pending_[1]) {
                case 0x51:
                    acc.x = read_s16(2) / 32768.0 * 16.0 * 9.8 - calib.acc_x_offset;
                    acc.y = read_s16(4) / 32768.0 * 16.0 * 9.8 - calib.acc_y_offset;
                    acc.z = read_s16(6) / 32768.0 * 16.0 * 9.8 - calib.acc_z_offset;
                    acc_time_s_ = receipt_s;
                    ++update.acc_frames;
                    break;
                case 0x52:
                    gyro.x = read_s16(2) / 32768.0 * 2000.0 / 180.0 * M_PI - calib.gyro_x_offset;
                    gyro.y = read_s16(4) / 32768.0 * 2000.0 / 180.0 * M_PI - calib.gyro_y_offset;
                    gyro.z = read_s16(6) / 32768.0 * 2000.0 / 180.0 * M_PI - calib.gyro_z_offset;
                    ++update.gyro_frames;
                    ++stats.gyro_frames;
                    // Copy at each gyro frame, before a later frame overwrites
                    // these fields. Freshness refers to host reception only.
                    update.samples.push_back({acc, gyro, angle,
                        receipt_s - acc_time_s_ <= freshness_s,
                        receipt_s - angle_time_s_ <= freshness_s});
                    break;
                case 0x53:
                    angle.r = read_s16(2) / 32768.0 * M_PI;
                    angle.p = read_s16(4) / 32768.0 * M_PI;
                    angle.y = read_s16(6) / 32768.0 * M_PI;
                    angle_time_s_ = receipt_s;
                    ++update.angle_frames;
                    break;
                default:
                    // Valid time/magnetic/other frames are consumed without publishing.
                    break;
            }
            pending_size_ = 0;
        }
        update.publishable = std::any_of(update.samples.begin(), update.samples.end(),
            [](const Sample& sample) { return sample.acceleration_valid; });
        update.orientation_valid = receipt_s - angle_time_s_ <= freshness_s;
        return update;
    }

    // 校准方法
    void calibrate(double new_acc_x_offset, double new_acc_y_offset, double new_acc_z_offset,
                   double new_gyro_x_offset, double new_gyro_y_offset, double new_gyro_z_offset)
    {
        calib.acc_x_offset = new_acc_x_offset;
        calib.acc_y_offset = new_acc_y_offset;
        calib.acc_z_offset = new_acc_z_offset;
        calib.gyro_x_offset = new_gyro_x_offset;
        calib.gyro_y_offset = new_gyro_y_offset;
        calib.gyro_z_offset = new_gyro_z_offset;
    }
    
    // 获取当前校准值
    void get_calibration(double& acc_x, double& acc_y, double& acc_z,
                        double& gyro_x, double& gyro_y, double& gyro_z)
    {
        acc_x = calib.acc_x_offset;
        acc_y = calib.acc_y_offset;
        acc_z = calib.acc_z_offset;
        gyro_x = calib.gyro_x_offset;
        gyro_y = calib.gyro_y_offset;
        gyro_z = calib.gyro_z_offset;
    }
};
