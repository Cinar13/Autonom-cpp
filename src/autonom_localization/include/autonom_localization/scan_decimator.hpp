#pragma once

#include <cstddef>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace autonom_localization
{

/// Keeps every Nth beam of a LaserScan. rf2o is single-threaded and scales with beam count; a dense
/// RPLiDAR S3 scan (3200 beams) saturates a core, while scan matching needs far fewer beams.
class ScanDecimator : public rclcpp::Node
{
public:
  explicit ScanDecimator(const rclcpp::NodeOptions & options);

private:
  void onScan(const sensor_msgs::msg::LaserScan & scan);

  std::size_t decimation_{1};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace autonom_localization
