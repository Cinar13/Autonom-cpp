#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "autonom_hardware/button_debouncer.hpp"
#include "autonom_hardware/gpio.hpp"

namespace autonom_hardware
{

/// Physical start button: a switch between `pin` (internal pull-up) and GND. Each debounced press publishes
/// std_msgs/Empty on race/start_signal; race_start_gate then applies the 5 s start delay and ignores repeats.
class StartButtonNode : public rclcpp::Node
{
public:
  explicit StartButtonNode(const rclcpp::NodeOptions & options);

private:
  void poll();

  std::unique_ptr<GpioInput> button_;
  std::optional<ButtonDebouncer> debouncer_;
  bool active_low_{true};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace autonom_hardware
