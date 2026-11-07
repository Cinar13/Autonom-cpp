#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "autonom_race/start_gate.hpp"

namespace autonom_race
{

/// Sits between Nav2 and the drive: forwards cmd_vel_in -> cmd_vel_out only while RACING and actively holds
/// the robot with zero twists otherwise. Publishes the race state (latched) so the loop patrol sends no Nav2
/// goal before the start delay has elapsed.
///
/// Start: `race/start_signal` (std_msgs/Empty, from a button or RC5 IR driver) or the `race/start` service.
/// Stop:  `race/stop_signal` or the `race/stop` service.
/// Time follows the node clock, so with use_sim_time the delay is measured in simulated seconds.
class RaceStartGateNode : public rclcpp::Node
{
public:
  explicit RaceStartGateNode(const rclcpp::NodeOptions & options);

private:
  [[nodiscard]] StartGate::Duration now() const;
  [[nodiscard]] bool clockValid() const;

  bool start(const char * source);
  bool stop(const char * source);
  void onTick();
  void onCmdVel(const geometry_msgs::msg::Twist & msg);
  void publishHold();
  void publishState();

  StartGate gate_;
  RaceState published_state_{RaceState::kIdle};
  bool auto_start_pending_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace autonom_race
