#include "autonom_race/race_start_gate_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace autonom_race
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

StartGate::Duration secondsToDuration(double seconds)
{
  return std::chrono::duration_cast<StartGate::Duration>(std::chrono::duration<double>(seconds));
}

}  // namespace

RaceStartGateNode::RaceStartGateNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("race_start_gate", options),
  gate_([this] {
      const auto delay = declareReadOnly<double>(
        *this, "start_delay", 5.0, "Seconds between the start signal and the first allowed motion (AC-3.2)");
      if (!std::isfinite(delay) || delay < 0.0) {
        throw std::invalid_argument("start_delay must be finite and non-negative");
      }
      return secondsToDuration(delay);
    }())
{
  const auto tick_rate = declareReadOnly<double>(
    *this, "tick_rate_hz", 50.0, "State update and zero-twist hold rate; bounds how late the release can be [Hz]");
  if (!std::isfinite(tick_rate) || tick_rate <= 0.0) {
    throw std::invalid_argument("tick_rate_hz must be positive");
  }
  auto_start_pending_ = declareReadOnly<bool>(
    *this, "auto_start", false, "Issue the start signal as soon as the clock is valid (simulation and bench tests)");

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", rclcpp::SystemDefaultsQoS());
  state_pub_ = create_publisher<std_msgs::msg::String>("race/state", rclcpp::QoS(1).reliable().transient_local());

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist & msg) { onCmdVel(msg); });
  start_sub_ = create_subscription<std_msgs::msg::Empty>(
    "race/start_signal", rclcpp::QoS(10).reliable(), [this](const std_msgs::msg::Empty &) { start("start signal"); });
  stop_sub_ = create_subscription<std_msgs::msg::Empty>(
    "race/stop_signal", rclcpp::QoS(10).reliable(), [this](const std_msgs::msg::Empty &) { stop("stop signal"); });

  start_srv_ = create_service<std_srvs::srv::Trigger>(
    "race/start",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = start("start service");
      response->message = toString(gate_.state());
    });
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "race/stop",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = stop("stop service");
      response->message = toString(gate_.state());
    });

  // Node clock: under use_sim_time the countdown runs in simulated seconds.
  tick_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / tick_rate)),
    [this] { onTick(); });

  publishState();
  RCLCPP_INFO(get_logger(), "Motion blocked until the start signal (then %.2f s delay)",
    get_parameter("start_delay").as_double());
}

StartGate::Duration RaceStartGateNode::now() const
{
  return StartGate::Duration(get_clock()->now().nanoseconds());
}

bool RaceStartGateNode::clockValid() const
{
  // Under use_sim_time the clock reads zero until the first /clock message; a countdown started then would
  // complete instantly when the clock jumps to the simulation time.
  return get_clock()->now().nanoseconds() > 0;
}

bool RaceStartGateNode::start(const char * source)
{
  if (!clockValid()) {
    RCLCPP_WARN(get_logger(), "Ignoring %s: clock not running yet", source);
    return false;
  }
  if (!gate_.requestStart(now())) {
    RCLCPP_WARN(get_logger(), "Ignoring %s while %s", source, toString(gate_.state()));
    return false;
  }
  RCLCPP_INFO(get_logger(), "Start (%s): holding still for %.2f s", source,
    std::chrono::duration<double>(gate_.remaining(now())).count());
  publishHold();
  publishState();
  return true;
}

bool RaceStartGateNode::stop(const char * source)
{
  if (!gate_.requestStop()) {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Stop (%s): motion blocked", source);
  publishHold();
  publishState();
  return true;
}

void RaceStartGateNode::onTick()
{
  if (auto_start_pending_ && clockValid()) {
    auto_start_pending_ = false;
    start("auto_start");
  }
  if (gate_.update(now()) != published_state_) {
    RCLCPP_INFO(get_logger(), "%s -> %s", toString(published_state_), toString(gate_.state()));
    publishState();
  }
  if (!gate_.motionAllowed()) {
    publishHold();
  }
}

void RaceStartGateNode::onCmdVel(const geometry_msgs::msg::Twist & msg)
{
  // Re-evaluate here too, so a command arriving between ticks is judged against the current time.
  gate_.update(now());
  if (gate_.motionAllowed()) {
    cmd_vel_pub_->publish(msg);
  }
}

void RaceStartGateNode::publishHold()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

void RaceStartGateNode::publishState()
{
  std_msgs::msg::String msg;
  published_state_ = gate_.state();
  msg.data = toString(published_state_);
  state_pub_->publish(msg);
}

}  // namespace autonom_race

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_race::RaceStartGateNode)
