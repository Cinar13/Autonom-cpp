#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "autonom_hardware/a4950_motor.hpp"
#include "autonom_hardware/diff_drive_kinematics.hpp"
#include "autonom_hardware/pid_controller.hpp"

namespace autonom_hardware
{

/// Closed-loop wheel velocity control: /cmd_vel -> per-wheel PID on encoder feedback -> A4950 PWM.
class MotorDriverNode : public rclcpp::Node
{
public:
  explicit MotorDriverNode(const rclcpp::NodeOptions & options);
  ~MotorDriverNode() override;

private:
  A4950Config declareMotorConfig(const std::string & prefix, int default_channel, int default_direction_pin);
  PidGains declarePidGains();
  rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter> & parameters);

  void onCmdVel(const geometry_msgs::msg::Twist & msg);
  void onJointState(const sensor_msgs::msg::JointState & msg);
  void controlLoop();
  void stopMotors();

  std::optional<DiffDriveKinematics> kinematics_;
  std::unique_ptr<A4950Motor> left_motor_;
  std::unique_ptr<A4950Motor> right_motor_;
  std::optional<PidController> left_pid_;
  std::optional<PidController> right_pid_;

  std::string left_joint_name_;
  std::string right_joint_name_;
  double max_wheel_speed_{0.0};
  double stop_velocity_threshold_{0.0};
  rclcpp::Duration cmd_vel_timeout_{0, 0};
  rclcpp::Duration feedback_timeout_{0, 0};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  BodyMotion commanded_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_STEADY_TIME};
  WheelPair measured_;
  rclcpp::Time last_feedback_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_STEADY_TIME};
  bool motors_active_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace autonom_hardware
