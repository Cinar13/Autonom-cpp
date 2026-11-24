#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "autonom_hardware/tb6612_motor.hpp"

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

/// Declares `<prefix>.pwm_chip|pwm_channel|in1_pin|in2_pin|pwm_frequency_hz|inverted` for one TB6612FNG channel.
inline Tb6612Config declareTb6612Config(
  rclcpp::Node & node, const std::string & prefix, int default_channel, int default_in1_pin, int default_in2_pin)
{
  Tb6612Config config;
  config.pwm_chip_path = declareParameter<std::string>(
    node, prefix + ".pwm_chip", "/sys/class/pwm/pwmchip0", "sysfs PWM chip driving the TB6612 PWMx input");
  const auto pwm_channel = declareParameter<std::int64_t>(
    node, prefix + ".pwm_channel", default_channel, "PWM channel on pwm_chip driving the TB6612 PWMx input");
  if (pwm_channel < 0 || pwm_channel > 15) {
    throw std::invalid_argument(prefix + ".pwm_channel must be in [0, 15]");
  }
  config.pwm_channel = static_cast<unsigned int>(pwm_channel);
  config.in1_pin = static_cast<unsigned int>(declarePin(node, prefix + ".in1_pin", default_in1_pin, "TB6612 xIN1 pin"));
  config.in2_pin = static_cast<unsigned int>(declarePin(node, prefix + ".in2_pin", default_in2_pin, "TB6612 xIN2 pin"));
  if (config.in1_pin == config.in2_pin) {
    throw std::invalid_argument(prefix + ".in1_pin and in2_pin must differ");
  }
  config.pwm_frequency_hz = declareParameter<double>(
    node, prefix + ".pwm_frequency_hz", 20000.0, "PWM carrier frequency; 20 kHz is inaudible and under the TB6612's 100 kHz");
  config.inverted = declareParameter<bool>(node, prefix + ".inverted", false, "Reverse motor polarity to match mounting");
  return config;
}

/// Declares the TB6612FNG `standby_pin` (STBY, shared by both channels; low = all outputs off).
inline unsigned int declareStandbyPin(rclcpp::Node & node, int default_pin)
{
  return static_cast<unsigned int>(declarePin(node, "standby_pin", default_pin, "TB6612 STBY pin (high = enabled)"));
}

}  // namespace autonom_hardware
