#ifndef LSLIDAR_TIMED_SCAN_H
#define LSLIDAR_TIMED_SCAN_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"

namespace lslidar_driver {

// N10 packets advance in hardware angle; ROS bearings advance clockwise.
// Use negative angular increments and positive time increments together.
// Sweep bounds are host reception estimates, not hardware timestamps.
template <typename Point>
bool make_timed_n10_scan(sensor_msgs::msg::LaserScan &scan,
                        const std::vector<Point> &points, int count,
                        const builtin_interfaces::msg::Time &sweep_start,
                        float sweep_duration, double allowed_min_deg,
                        double allowed_max_deg, bool truncated,
                        const int *crop_min_deg, const int *crop_max_deg,
                        int crop_count) {
  if (count < 2 || static_cast<size_t>(count) > points.size() ||
      !std::isfinite(sweep_duration) || sweep_duration <= 0.f) {
    return false;
  }
  const double step = 2.0 * std::acos(-1.0) / count;
  scan.header.stamp = sweep_start;
  scan.angle_min = 0.f;
  scan.angle_increment = static_cast<float>(-step);
  scan.angle_max = static_cast<float>(-(count - 1) * step);
  scan.scan_time = sweep_duration;
  scan.time_increment = sweep_duration / count;
  scan.ranges.assign(count, std::numeric_limits<float>::infinity());
  scan.intensities.assign(count, 0.f);

  for (int i = 0; i < count; ++i) {
    const auto &point = points[i];
    if (!std::isfinite(point.degree) || !std::isfinite(point.range) ||
        point.degree < 0.0 || point.degree >= 360.0 ||
        point.range < scan.range_min || point.range > scan.range_max) {
      continue;
    }
    const int index = static_cast<int>(std::lround(point.degree * count / 360.0));
    // A late sample rounding to 360 degrees belongs at the end, never at time 0.
    // Omit this duplicate seam sample rather than wrap it to the first bin.
    if (index < 0 || index >= count) continue;
    double bearing = std::fmod(360.0 - point.degree, 360.0);
    double allowed_bearing = bearing;
    if (allowed_bearing < allowed_min_deg) allowed_bearing += 360.0;
    if (allowed_bearing < allowed_min_deg || allowed_bearing > allowed_max_deg)
      continue;
    bool masked = false;
    if (truncated) {
      for (int j = 0; j < crop_count; ++j) {
        if (bearing >= crop_min_deg[j] && bearing <= crop_max_deg[j]) {
          masked = true;
          break;
        }
      }
    }
    if (masked) continue;
    scan.ranges[index] = static_cast<float>(point.range);
    scan.intensities[index] = static_cast<float>(point.intensity);
  }
  return true;
}

}  // namespace lslidar_driver
#endif
