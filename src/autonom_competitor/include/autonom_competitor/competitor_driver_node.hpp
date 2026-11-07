#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "autonom_competitor/lane_follower.hpp"

namespace autonom_competitor
{

/// Simulated competitor (docs/07): drives its lane around the track with a LaneFollower on the ground-truth
/// odometry of its Gazebo model. Holds still until /race/state is RACING, like the main robot's start gate
/// (5 s start delay, AC-3.2), and stops again when the race stops. It follows a pre-programmed path and does not
/// steer around anything, but it stops while another robot (ground-truth odometry topics `others`) is right ahead
/// of it, so it never rams (docs/06 section 2).
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
  double give_way_distance_{0.35};
  double give_way_half_width_{0.18};
  struct Other
  {
    std::optional<Point> position;
    rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  };
  std::vector<Other> others_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_subs_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr race_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonom_competitor
