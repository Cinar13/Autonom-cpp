#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace autonom_hardware
{

template<typename T>
T declareParameter(
  rclcpp::Node & node, const std::string & name, const T & default_value, const std::string & description,
  bool read_only = true)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = read_only;
  return node.declare_parameter<T>(name, default_value, descriptor);
}

inline int declarePin(rclcpp::Node & node, const std::string & name, int default_pin, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description + " (GPIO line offset on the chip, BCM numbering on RPi 5)";
  descriptor.read_only = true;
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = 0;
  descriptor.integer_range[0].to_value = 63;
  descriptor.integer_range[0].step = 1;
  return static_cast<int>(node.declare_parameter<std::int64_t>(name, default_pin, descriptor));
}

/// Declares a 6-element covariance diagonal [x, y, z, roll, pitch, yaw] and expands it to a row-major 6x6 matrix.
inline std::array<double, 36> declareCovarianceDiagonal(
  rclcpp::Node & node, const std::string & name, const std::array<double, 6> & default_diagonal)
{
  const auto diagonal = declareParameter<std::vector<double>>(
    node, name, std::vector<double>(default_diagonal.begin(), default_diagonal.end()),
    "Diagonal [x, y, z, roll, pitch, yaw] of the 6x6 covariance");
  if (diagonal.size() != 6) {
    throw std::invalid_argument(name + " must contain exactly 6 values");
  }
  std::array<double, 36> covariance{};
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.0) {
      throw std::invalid_argument(name + " values must be finite and non-negative");
    }
    covariance[i * 7] = diagonal[i];
  }
  return covariance;
}

}  // namespace autonom_hardware
