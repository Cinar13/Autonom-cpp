#include "autonom_hardware/sim_wheel_odometry_node.hpp"

#include <cmath>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/encoder_model.hpp"
#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{

SimWheelOdometryNode::SimWheelOdometryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wheel_odometry", options), last_stamp_(0, 0, get_clock()->get_clock_type())
{
  const auto wheel_radius = declareParameter<double>(*this, "wheel_radius", 0.0325, "Wheel radius [m]");
  const auto wheel_separation =
    declareParameter<double>(*this, "wheel_separation", 0.17, "Distance between wheel contact points [m]");
  ticks_per_revolution_ = declareParameter<std::int64_t>(
    *this, "ticks_per_revolution", 1400, "Encoder counts per wheel revolution after x4 decoding (incl. gearbox)");
  odom_frame_id_ = declareParameter<std::string>(*this, "odom_frame_id", "odom", "Odometry frame");
  base_frame_id_ = declareParameter<std::string>(*this, "base_frame_id", "base_footprint", "Robot base frame");
  left_joint_name_ =
    declareParameter<std::string>(*this, "left_joint_name", "left_wheel_joint", "Left wheel joint name");
  right_joint_name_ =
    declareParameter<std::string>(*this, "right_joint_name", "right_wheel_joint", "Right wheel joint name");
  pose_covariance_ = declareCovarianceDiagonal(*this, "pose_covariance_diagonal", {1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2});
  twist_covariance_ = declareCovarianceDiagonal(*this, "twist_covariance_diagonal", {2e-3, 1e-4, 1e6, 1e6, 1e6, 2e-2});
  if (ticks_per_revolution_ <= 0) {
    throw std::invalid_argument("ticks_per_revolution must be positive");
  }
  kinematics_.emplace(wheel_radius, wheel_separation);
  radians_per_tick_ = 2.0 * M_PI / static_cast<double>(ticks_per_revolution_);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", rclcpp::SystemDefaultsQoS());
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & msg) { onJointStates(msg); });
  RCLCPP_INFO(get_logger(), "Simulated encoders: %ld counts per wheel revolution", static_cast<long>(ticks_per_revolution_));
}

void SimWheelOdometryNode::onJointStates(const sensor_msgs::msg::JointState & msg)
{
  std::optional<double> left;
  std::optional<double> right;
  for (std::size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    if (msg.name[i] == left_joint_name_) {
      left = msg.position[i];
    } else if (msg.name[i] == right_joint_name_) {
      right = msg.position[i];
    }
  }
  if (!left || !right) {
    RCLCPP_WARN_ONCE(get_logger(), "joint_states has no '%s' / '%s' positions", left_joint_name_.c_str(),
      right_joint_name_.c_str());
    return;
  }
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const std::int64_t left_ticks = angleToCounts(*left, ticks_per_revolution_);
  const std::int64_t right_ticks = angleToCounts(*right, ticks_per_revolution_);
  if (!initialised_) {
    last_left_ticks_ = left_ticks;
    last_right_ticks_ = right_ticks;
    last_stamp_ = stamp;
    initialised_ = true;
    return;
  }
  const double dt = (stamp - last_stamp_).seconds();
  if (dt <= 0.0) {
    return;
  }
  const WheelPair wheel_delta{
    static_cast<double>(left_ticks - last_left_ticks_) * radians_per_tick_,
    static_cast<double>(right_ticks - last_right_ticks_) * radians_per_tick_};
  last_left_ticks_ = left_ticks;
  last_right_ticks_ = right_ticks;
  last_stamp_ = stamp;

  const BodyMotion displacement = kinematics_->toBody(wheel_delta);
  pose_.integrate(displacement);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = msg.header.stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  odom.pose.pose.orientation.z = std::sin(0.5 * pose_.theta);
  odom.pose.pose.orientation.w = std::cos(0.5 * pose_.theta);
  odom.pose.covariance = pose_covariance_;
  odom.twist.twist.linear.x = displacement.linear / dt;
  odom.twist.twist.angular.z = displacement.angular / dt;
  odom.twist.covariance = twist_covariance_;
  odom_pub_->publish(odom);
}

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::SimWheelOdometryNode)
