#include "autonom_hardware/wheel_odometry_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/gpio.hpp"
#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{
namespace
{

constexpr auto kWarnThrottleMs = 5000;

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

WheelOdometryNode::WheelOdometryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wheel_odometry", options)
{
  const auto gpio_chip =
    declareParameter<std::string>(*this, "gpio_chip", "pinctrl-rp1", "GPIO chip name, label, path or number");
  const auto wheel_radius = declareParameter<double>(*this, "wheel_radius", 0.0325, "Wheel radius [m]");
  const auto wheel_separation =
    declareParameter<double>(*this, "wheel_separation", 0.17, "Distance between wheel contact points [m]");
  const auto ticks_per_revolution = declareParameter<std::int64_t>(
    *this, "ticks_per_revolution", 1372, "Encoder counts per wheel revolution after x4 decoding (incl. gearbox)");
  const auto publish_frequency =
    declareParameter<double>(*this, "publish_frequency_hz", 50.0, "Odometry and joint state publish rate [Hz]");
  const auto publish_tf = declareParameter<bool>(
    *this, "publish_tf", false, "Broadcast odom->base TF (keep false when robot_localization owns it)");
  odom_frame_id_ = declareParameter<std::string>(*this, "odom_frame_id", "odom", "Odometry frame");
  base_frame_id_ = declareParameter<std::string>(*this, "base_frame_id", "base_footprint", "Robot base frame");
  left_joint_name_ =
    declareParameter<std::string>(*this, "left_joint_name", "left_wheel_joint", "Left wheel joint name");
  right_joint_name_ =
    declareParameter<std::string>(*this, "right_joint_name", "right_wheel_joint", "Right wheel joint name");
  pose_covariance_ = declareCovarianceDiagonal(*this, "pose_covariance_diagonal", {1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2});
  twist_covariance_ = declareCovarianceDiagonal(*this, "twist_covariance_diagonal", {2e-3, 1e-4, 1e6, 1e6, 1e6, 2e-2});

  if (ticks_per_revolution <= 0) {
    throw std::invalid_argument("ticks_per_revolution must be positive");
  }
  if (!(publish_frequency > 0.0)) {
    throw std::invalid_argument("publish_frequency_hz must be positive");
  }
  kinematics_.emplace(wheel_radius, wheel_separation);
  radians_per_tick_ = 2.0 * M_PI / static_cast<double>(ticks_per_revolution);

  const QuadratureEncoderConfig left_config = declareEncoderConfig("left_encoder", 17, 27);
  const QuadratureEncoderConfig right_config = declareEncoderConfig("right_encoder", 23, 24);
  try {
    const GpioChipPtr chip = openGpioChip(gpio_chip);
    left_encoder_ = std::make_unique<QuadratureEncoder>(chip, left_config, "autonom_left_encoder");
    right_encoder_ = std::make_unique<QuadratureEncoder>(chip, right_config, "autonom_right_encoder");
  } catch (const std::exception & error) {
    RCLCPP_FATAL(get_logger(), "Encoder hardware initialisation failed: %s", error.what());
    throw;
  }
  last_left_ticks_ = left_encoder_->count();
  last_right_ticks_ = right_encoder_->count();
  last_update_time_ = steady_clock_.now();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel/odom", rclcpp::SystemDefaultsQoS());
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS());
  if (publish_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  update_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency), [this] { update(); });

  RCLCPP_INFO(get_logger(), "Wheel odometry running at %.1f Hz (%ld ticks/rev)", publish_frequency,
    static_cast<long>(ticks_per_revolution));
}

QuadratureEncoderConfig WheelOdometryNode::declareEncoderConfig(
  const std::string & prefix, int default_pin_a, int default_pin_b)
{
  QuadratureEncoderConfig config;
  config.pin_a = static_cast<unsigned int>(declarePin(*this, prefix + ".pin_a", default_pin_a, "Encoder channel A"));
  config.pin_b = static_cast<unsigned int>(declarePin(*this, prefix + ".pin_b", default_pin_b, "Encoder channel B"));
  config.inverted = declareParameter<bool>(*this, prefix + ".inverted", false, "Reverse count direction");
  config.pull_up = declareParameter<bool>(*this, prefix + ".pull_up", true, "Enable internal pull-ups (open-collector)");
  return config;
}

void WheelOdometryNode::update()
{
  const rclcpp::Time now = steady_clock_.now();
  const double dt = (now - last_update_time_).seconds();
  if (dt <= 0.0) {
    return;
  }
  last_update_time_ = now;

  const std::int64_t left_ticks = left_encoder_->count();
  const std::int64_t right_ticks = right_encoder_->count();
  const WheelPair wheel_delta{
    static_cast<double>(left_ticks - last_left_ticks_) * radians_per_tick_,
    static_cast<double>(right_ticks - last_right_ticks_) * radians_per_tick_};
  last_left_ticks_ = left_ticks;
  last_right_ticks_ = right_ticks;

  const BodyMotion displacement = kinematics_->toBody(wheel_delta);
  pose_.integrate(displacement);

  if (!left_encoder_->healthy() || !right_encoder_->healthy()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Encoder event thread stopped; odometry frozen");
  }
  const std::uint64_t missed = left_encoder_->missedTransitions() + right_encoder_->missedTransitions();
  if (missed != reported_missed_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), steady_clock_, kWarnThrottleMs, "Encoders reported %lu missed edges in total",
      static_cast<unsigned long>(missed));
    reported_missed_ = missed;
  }

  const rclcpp::Time stamp = get_clock()->now();
  publishOdometry(stamp, {displacement.linear / dt, displacement.angular / dt});
  publishJointStates(stamp, {wheel_delta.left / dt, wheel_delta.right / dt});
}

void WheelOdometryNode::publishOdometry(const rclcpp::Time & stamp, const BodyMotion & velocity)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  odom.pose.pose.orientation = yawToQuaternion(pose_.theta);
  odom.pose.covariance = pose_covariance_;
  odom.twist.twist.linear.x = velocity.linear;
  odom.twist.twist.angular.z = velocity.angular;
  odom.twist.covariance = twist_covariance_;

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
  odom_pub_->publish(odom);
}

void WheelOdometryNode::publishJointStates(const rclcpp::Time & stamp, const WheelPair & velocity)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name = {left_joint_name_, right_joint_name_};
  joint_state.position = {
    static_cast<double>(last_left_ticks_) * radians_per_tick_, static_cast<double>(last_right_ticks_) * radians_per_tick_};
  joint_state.velocity = {velocity.left, velocity.right};
  joint_state_pub_->publish(joint_state);
}

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::WheelOdometryNode)
