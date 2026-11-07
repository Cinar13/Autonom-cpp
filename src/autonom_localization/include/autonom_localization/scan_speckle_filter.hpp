#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "autonom_localization/speckle_filter.hpp"

namespace autonom_localization
{

/// Republishes a LaserScan with unsupported single returns replaced by +inf (no return), so Nav2 costmaps
/// do not turn one noisy beam into a lethal obstacle. slam_toolbox keeps the raw scan.
class ScanSpeckleFilter : public rclcpp::Node
{
public:
  explicit ScanSpeckleFilter(const rclcpp::NodeOptions & options);

private:
  void onScan(sensor_msgs::msg::LaserScan::UniquePtr scan);

  SpeckleFilterSettings settings_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace autonom_localization
