#include "autonom_competitor/competitor_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <rclcpp/create_timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace autonom_competitor
{

CompetitorDriverNode::CompetitorDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("competitor_driver", options)
{
  // Closed centre line of the track in world coordinates, in the main robot's default race direction.
  const auto xs = declare_parameter<std::vector<double>>("path_x", std::vector<double>{});
  const auto ys = declare_parameter<std::vector<double>>("path_y", std::vector<double>{});
  if (xs.size() != ys.size() || xs.size() < 3) {
    throw std::invalid_argument("path_x and path_y must have the same length, at least 3 points");
  }
  std::vector<Point> path;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    path.push_back({xs[i], ys[i]});
  }
  // Race the other way round (main robot spawned facing the opposite direction).
  if (declare_parameter<bool>("reverse", false)) {
    std::reverse(path.begin(), path.end());
  }
  LaneFollowerSettings s;
  s.speed = declare_parameter<double>("speed", s.speed);
  s.lane_offset = declare_parameter<double>("lane_offset", s.lane_offset);
  s.merge_distance = declare_parameter<double>("merge_distance", s.merge_distance);
  s.lookahead = declare_parameter<double>("lookahead", s.lookahead);
  s.max_angular = declare_parameter<double>("max_angular", s.max_angular);
  s.turn_in_place_angle = declare_parameter<double>("turn_in_place_angle", s.turn_in_place_angle);
  s.search_window = declare_parameter<double>("search_window", s.search_window);
  follower_ = std::make_unique<LaneFollower>(std::move(path), s);
  wait_for_race_start_ = declare_parameter<bool>("wait_for_race_start", true);
  // Give way: stop while another robot's centre is within give_way_distance ahead and give_way_half_width of the
  // heading line. `others`: ground-truth odometry topics of the other robots (world frame).
  give_way_distance_ = declare_parameter<double>("give_way_distance", give_way_distance_);
  give_way_half_width_ = declare_parameter<double>("give_way_half_width", give_way_half_width_);
  const auto other_topics = declare_parameter<std::vector<std::string>>("others", std::vector<std::string>{});
  others_.resize(other_topics.size());
  for (std::size_t i = 0; i < other_topics.size(); ++i) {
    other_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      other_topics[i], rclcpp::SensorDataQoS(), [this, i](const nav_msgs::msg::Odometry & msg) {
        others_[i].position = Point{msg.pose.pose.position.x, msg.pose.pose.position.y};
        others_[i].stamp = steady_clock_.now();
      }));
  }
  const auto race_state_topic = declare_parameter<std::string>("race_state_topic", "/race/state");
  const double rate = declare_parameter<double>("rate", 20.0);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
      const auto & q = msg.pose.pose.orientation;
      pose_ = Pose2D{msg.pose.pose.position.x, msg.pose.pose.position.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    });
  race_state_sub_ = create_subscription<std_msgs::msg::String>(
    race_state_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String & msg) { race_state_ = msg.data; });
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Node clock: simulated time in Gazebo, like the start gate's delay.
  timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { onTick(); });
  RCLCPP_INFO(get_logger(), "Lane %+.2f m at %.2f m/s on a %.2f m loop", s.lane_offset, s.speed, follower_->length());
}

void CompetitorDriverNode::publish(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_pub_->publish(cmd);
}

void CompetitorDriverNode::onTick()
{
  const bool racing = !wait_for_race_start_ || race_state_ == "RACING";
  if (!racing || !pose_) {
    if (driving_) {
      RCLCPP_INFO(get_logger(), "Race state %s: stopping", race_state_.c_str());
      driving_ = false;
    }
    publish(0.0, 0.0);
    return;
  }
  if (!driving_) {
    RCLCPP_INFO(get_logger(), "Driving");
    driving_ = true;
  }
  std::vector<Point> nearby;
  const auto now = steady_clock_.now();
  for (const auto & other : others_) {
    if (other.position && (now - other.stamp).seconds() < 1.0) {
      nearby.push_back(*other.position);
    }
  }
  if (robotAhead(*pose_, nearby, give_way_distance_, give_way_half_width_)) {
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 5000, "Giving way to a robot ahead");
    publish(0.0, 0.0);
    return;
  }
  const auto cmd = follower_->update(*pose_);
  publish(cmd.linear, cmd.angular);
}

}  // namespace autonom_competitor

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_competitor::CompetitorDriverNode)
