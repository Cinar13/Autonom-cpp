#include "autonom_localization/scan_decimator.hpp"

#include <cstdint>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

namespace autonom_localization
{

ScanDecimator::ScanDecimator(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_decimator", options)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Keep one beam out of every `decimation` beams";
  descriptor.read_only = true;
  const auto decimation = declare_parameter<std::int64_t>("decimation", 4, descriptor);
  if (decimation < 1) {
    throw std::invalid_argument("decimation must be >= 1");
  }
  decimation_ = static_cast<std::size_t>(decimation);

  // Reliable output: rf2o subscribes with the default (reliable) QoS.
  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_out", rclcpp::QoS(5));
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_in", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & scan) { onScan(scan); });
}

void ScanDecimator::onScan(const sensor_msgs::msg::LaserScan & scan)
{
  sensor_msgs::msg::LaserScan decimated;
  decimated.header = scan.header;
  decimated.angle_min = scan.angle_min;
  decimated.angle_increment = scan.angle_increment * static_cast<float>(decimation_);
  decimated.time_increment = scan.time_increment * static_cast<float>(decimation_);
  decimated.scan_time = scan.scan_time;
  decimated.range_min = scan.range_min;
  decimated.range_max = scan.range_max;

  const std::size_t beams = (scan.ranges.size() + decimation_ - 1) / decimation_;
  decimated.ranges.reserve(beams);
  const bool has_intensities = scan.intensities.size() == scan.ranges.size();
  if (has_intensities) {
    decimated.intensities.reserve(beams);
  }
  for (std::size_t i = 0; i < scan.ranges.size(); i += decimation_) {
    decimated.ranges.push_back(scan.ranges[i]);
    if (has_intensities) {
      decimated.intensities.push_back(scan.intensities[i]);
    }
  }
  if (decimated.ranges.empty()) {
    return;
  }
  decimated.angle_max = decimated.angle_min + decimated.angle_increment * static_cast<float>(decimated.ranges.size() - 1);
  publisher_->publish(decimated);
}

}  // namespace autonom_localization

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_localization::ScanDecimator)
