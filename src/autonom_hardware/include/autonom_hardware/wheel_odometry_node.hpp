#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "autonom_hardware/diff_drive_kinematics.hpp"
#include "autonom_hardware/quadrature_encoder.hpp"

namespace autonom_hardware
{

/// Publishes /wheel/odom and wheel joint_states from interrupt-driven quadrature encoders.
class WheelOdometryNode : public rclcpp::Node
{
public:
  explicit WheelOdometryNode(const rclcpp::NodeOptions & options);

private:
  QuadratureEncoderConfig declareEncoderConfig(const std::string & prefix, int default_pin_a, int default_pin_b);
  void update();
  void publishOdometry(const rclcpp::Time & stamp, const BodyMotion & velocity);
  void publishJointStates(const rclcpp::Time & stamp, const WheelPair & velocity);

  std::optional<DiffDriveKinematics> kinematics_;
  std::unique_ptr<QuadratureEncoder> left_encoder_;
  std::unique_ptr<QuadratureEncoder> right_encoder_;
  double radians_per_tick_{0.0};

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_STEADY_TIME};
  std::int64_t last_left_ticks_{0};
  std::int64_t last_right_ticks_{0};
  std::uint64_t reported_missed_{0};
  Pose2D pose_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace autonom_hardware
