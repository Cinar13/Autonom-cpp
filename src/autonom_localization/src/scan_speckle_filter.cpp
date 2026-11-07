#include "autonom_localization/scan_speckle_filter.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace autonom_localization
{
namespace
{

template<typename T>
T declareReadOnly(rclcpp::Node & node, const std::string & name, const T & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, default_value, descriptor);
}

}  // namespace

ScanSpeckleFilter::ScanSpeckleFilter(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_speckle_filter", options)
{
  const auto window = declareReadOnly<std::int64_t>(*this, "window", 3, "Beams inspected on each side of a return");
  const auto min_neighbors = declareReadOnly<std::int64_t>(
    *this, "min_neighbors", 2, "Neighbours at a similar range needed to keep a return");
  const auto abs_tolerance = declareReadOnly<double>(
    *this, "abs_tolerance", 0.05, "Range difference that still counts as the same surface [m]");
  const auto rel_tolerance = declareReadOnly<double>(
    *this, "rel_tolerance", 0.02, "Extra range difference per metre of range (oblique walls far away)");
  if (window < 1 || min_neighbors < 1 || min_neighbors > 2 * window) {
    throw std::invalid_argument("need window >= 1 and 1 <= min_neighbors <= 2 * window");
  }
  if (!(abs_tolerance >= 0.0) || !(rel_tolerance >= 0.0)) {
    throw std::invalid_argument("tolerances must be non-negative");
  }
  settings_.window = static_cast<std::size_t>(window);
  settings_.min_neighbors = static_cast<std::size_t>(min_neighbors);
  settings_.abs_tolerance = static_cast<float>(abs_tolerance);
  settings_.rel_tolerance = static_cast<float>(rel_tolerance);

  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_out", rclcpp::SensorDataQoS());
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_in", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::UniquePtr scan) { onScan(std::move(scan)); });
}

void ScanSpeckleFilter::onScan(sensor_msgs::msg::LaserScan::UniquePtr scan)
{
  for (const auto index : findSpeckles(scan->ranges, scan->range_min, scan->range_max, settings_)) {
    scan->ranges[index] = std::numeric_limits<float>::infinity();
  }
  publisher_->publish(std::move(scan));
}

}  // namespace autonom_localization

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_localization::ScanSpeckleFilter)
