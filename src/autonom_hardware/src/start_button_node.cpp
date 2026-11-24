#include "autonom_hardware/start_button_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{
namespace
{

constexpr auto kWarnThrottleMs = 5000;

}  // namespace

StartButtonNode::StartButtonNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("start_button", options)
{
  const auto gpio_chip =
    declareParameter<std::string>(*this, "gpio_chip", "pinctrl-rp1", "GPIO chip name, label, path or number");
  const auto pin = static_cast<unsigned int>(declarePin(*this, "pin", 16, "Start button input (other side to GND)"));
  const auto pull_up = declareParameter<bool>(*this, "pull_up", true, "Enable the internal pull-up");
  active_low_ = declareParameter<bool>(*this, "active_low", true, "Pressed reads low (switch to GND)");
  const auto poll_period_ms =
    declareParameter<std::int64_t>(*this, "poll_period_ms", 5, "Button sampling period [ms]");
  const auto debounce_ms =
    declareParameter<std::int64_t>(*this, "debounce_ms", 30, "Level must be stable this long to count [ms]");
  if (poll_period_ms <= 0 || debounce_ms <= 0) {
    throw std::invalid_argument("poll_period_ms and debounce_ms must be positive");
  }
  debouncer_.emplace(static_cast<int>(std::max<std::int64_t>(1, debounce_ms / poll_period_ms)));

  try {
    button_ = std::make_unique<GpioInput>(openGpioChip(gpio_chip), pin, "autonom_start_button", pull_up);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(get_logger(), "Start button initialisation failed: %s", error.what());
    throw;
  }

  start_pub_ = create_publisher<std_msgs::msg::Empty>("race/start_signal", rclcpp::QoS(10).reliable());
  poll_timer_ = create_wall_timer(std::chrono::milliseconds(poll_period_ms), [this] { poll(); });
  RCLCPP_INFO(get_logger(), "Start button on GPIO%u (%s), publishing race/start_signal", pin,
    active_low_ ? "active low" : "active high");
}

void StartButtonNode::poll()
{
  const int level = button_->read();
  if (level < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Failed to read the start button");
    return;
  }
  const bool was_armed = debouncer_->armed();
  if (debouncer_->update((level == 0) == active_low_)) {
    RCLCPP_INFO(get_logger(), "Start button pressed");
    start_pub_->publish(std_msgs::msg::Empty());
  }
  if (!was_armed && !debouncer_->armed()) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs,
      "Start button reads pressed; release it (or check the wiring) before it can start the race");
  }
}

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::StartButtonNode)
