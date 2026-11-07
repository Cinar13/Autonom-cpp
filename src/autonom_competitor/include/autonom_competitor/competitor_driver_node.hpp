#pragma once

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonom_competitor/lane_follower.hpp"

namespace autonom_competitor
{

/// Simulated competitor (docs/07): drives its lane around the track with a LaneFollower on the ground-truth
/// odometry of its Gazebo model. Holds still until /race/state is RACING, like the main robot's start gate
/// (5 s start delay, AC-3.2), and stops again when the race stops. It never avoids anything: a pre-programmed path.
class CompetitorDriverNode : public rclcpp::Node
{
public:
  explicit CompetitorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTick();
  void publish(double linear, double angular);

  std::unique_ptr<LaneFollower> follower_;
  bool wait_for_race_start_{true};
  std::string race_state_{"UNKNOWN"};
  std::optional<Pose2D> pose_;
  bool driving_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr race_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonom_competitor
