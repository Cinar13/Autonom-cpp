#include "autonom_hardware/motor_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{
namespace
{

constexpr auto kWarnThrottleMs = 2000;

}  // namespace

MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("motor_driver", options)
{
  const auto gpio_chip = declareParameter<std::string>(
    *this, "gpio_chip", "pinctrl-rp1", "GPIO chip name, label, path or number for the direction pins");
  const auto wheel_radius = declareParameter<double>(*this, "wheel_radius", 0.0325, "Wheel radius [m]");
  const auto wheel_separation =
    declareParameter<double>(*this, "wheel_separation", 0.17, "Distance between wheel contact points [m]");
  const auto control_frequency =
    declareParameter<double>(*this, "control_frequency_hz", 50.0, "Velocity control loop rate [Hz]");
  max_wheel_speed_ = declareParameter<double>(*this, "max_wheel_speed", 45.03, "Wheel speed saturation [rad/s]");
  stop_velocity_threshold_ = declareParameter<double>(
    *this, "stop_velocity_threshold", 0.05, "Wheel targets below this magnitude coast the motors [rad/s]");
  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(declareParameter<double>(
    *this, "cmd_vel_timeout", 0.5, "Coast the motors when no cmd_vel arrives within this period [s]"));
  feedback_timeout_ = rclcpp::Duration::from_seconds(declareParameter<double>(
    *this, "feedback_timeout", 0.25, "Coast the motors when encoder feedback is older than this [s]"));
  left_joint_name_ =
    declareParameter<std::string>(*this, "left_joint_name", "left_wheel_joint", "Left wheel joint in joint_states");
  right_joint_name_ =
    declareParameter<std::string>(*this, "right_joint_name", "right_wheel_joint", "Right wheel joint in joint_states");

  if (!(control_frequency > 0.0) || !(max_wheel_speed_ > 0.0) || cmd_vel_timeout_.nanoseconds() <= 0 ||
    feedback_timeout_.nanoseconds() <= 0)
  {
    throw std::invalid_argument("control_frequency_hz, max_wheel_speed and timeouts must be positive");
  }

  kinematics_.emplace(wheel_radius, wheel_separation);
  const PidGains gains = declarePidGains();
  left_pid_.emplace(gains);
  right_pid_.emplace(gains);

  const A4950Config left_config = declareMotorConfig("left_motor", 0, 5);
  const A4950Config right_config = declareMotorConfig("right_motor", 1, 6);
  try {
    const GpioChipPtr chip = openGpioChip(gpio_chip);
    left_motor_ = std::make_unique<A4950Motor>(chip, left_config, "autonom_left_motor");
    right_motor_ = std::make_unique<A4950Motor>(chip, right_config, "autonom_right_motor");
  } catch (const std::exception & error) {
    RCLCPP_FATAL(get_logger(), "Motor hardware initialisation failed: %s", error.what());
    throw;
  }

  parameter_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) { return onParametersChanged(parameters); });

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist & msg) { onCmdVel(msg); });
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & msg) { onJointState(msg); });
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_frequency), [this] { controlLoop(); });

  RCLCPP_INFO(get_logger(), "A4950 motor driver running at %.1f Hz", control_frequency);
}

MotorDriverNode::~MotorDriverNode()
{
  control_timer_.reset();
  stopMotors();
}

A4950Config MotorDriverNode::declareMotorConfig(
  const std::string & prefix, int default_channel, int default_direction_pin)
{
  A4950Config config;
  config.pwm_chip_path = declareParameter<std::string>(
    *this, prefix + ".pwm_chip", "/sys/class/pwm/pwmchip0", "sysfs PWM chip driving IN1");
  const auto pwm_channel = declareParameter<std::int64_t>(
    *this, prefix + ".pwm_channel", default_channel, "PWM channel on pwm_chip driving IN1");
  if (pwm_channel < 0 || pwm_channel > 15) {
    throw std::invalid_argument(prefix + ".pwm_channel must be in [0, 15]");
  }
  config.pwm_channel = static_cast<unsigned int>(pwm_channel);
  config.direction_pin =
    static_cast<unsigned int>(declarePin(*this, prefix + ".direction_pin", default_direction_pin, "A4950 IN2 pin"));
  config.pwm_frequency_hz = declareParameter<double>(
    *this, prefix + ".pwm_frequency_hz", 20000.0, "PWM carrier frequency; 20 kHz keeps it above audible range");
  config.inverted =
    declareParameter<bool>(*this, prefix + ".inverted", false, "Reverse motor polarity to match mounting");
  return config;
}

PidGains MotorDriverNode::declarePidGains()
{
  PidGains gains;
  gains.kp = declareParameter<double>(*this, "pid.kp", 0.04, "Proportional gain [effort per rad/s]", false);
  gains.ki = declareParameter<double>(*this, "pid.ki", 0.4, "Integral gain [effort per rad]", false);
  gains.kd = declareParameter<double>(*this, "pid.kd", 0.0, "Derivative gain [effort per rad/s^2]", false);
  gains.kf = declareParameter<double>(*this, "pid.kf", 0.022, "Feed-forward [effort per rad/s of setpoint]", false);
  gains.integral_limit =
    declareParameter<double>(*this, "pid.integral_limit", 0.5, "Integral term bound [effort]", false);
  gains.output_limit = declareParameter<double>(*this, "pid.output_limit", 1.0, "Max PWM effort in [0, 1]", false);
  return gains;
}

rcl_interfaces::msg::SetParametersResult MotorDriverNode::onParametersChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  PidGains gains = left_pid_->gains();
  bool gains_changed = false;
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    double * target = name == "pid.kp" ? &gains.kp :
      name == "pid.ki" ? &gains.ki :
      name == "pid.kd" ? &gains.kd :
      name == "pid.kf" ? &gains.kf :
      name == "pid.integral_limit" ? &gains.integral_limit :
      name == "pid.output_limit" ? &gains.output_limit :
      nullptr;
    if (target != nullptr) {
      *target = parameter.as_double();
      gains_changed = true;
    }
  }
  if (!gains_changed) {
    return result;
  }
  if (gains.output_limit > 1.0) {
    result.successful = false;
    result.reason = "pid.output_limit must not exceed 1.0";
    return result;
  }
  try {
    left_pid_->setGains(gains);
    right_pid_->setGains(gains);
  } catch (const std::invalid_argument & error) {
    result.successful = false;
    result.reason = error.what();
  }
  return result;
}

void MotorDriverNode::onCmdVel(const geometry_msgs::msg::Twist & msg)
{
  if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.angular.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Ignoring non-finite cmd_vel");
    return;
  }
  commanded_ = {msg.linear.x, msg.angular.z};
  last_cmd_time_ = steady_clock_.now();
}

void MotorDriverNode::onJointState(const sensor_msgs::msg::JointState & msg)
{
  if (msg.velocity.size() != msg.name.size()) {
    return;
  }
  std::optional<double> left;
  std::optional<double> right;
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == left_joint_name_) {
      left = msg.velocity[i];
    } else if (msg.name[i] == right_joint_name_) {
      right = msg.velocity[i];
    }
  }
  if (left && right && std::isfinite(*left) && std::isfinite(*right)) {
    measured_ = {*left, *right};
    last_feedback_time_ = steady_clock_.now();
  }
}

void MotorDriverNode::controlLoop()
{
  const rclcpp::Time now = steady_clock_.now();
  const double dt = (now - last_control_time_).seconds();
  last_control_time_ = now;

  if (now - last_cmd_time_ > cmd_vel_timeout_) {
    stopMotors();
    return;
  }

  const WheelPair target = DiffDriveKinematics::limit(kinematics_->toWheels(commanded_), max_wheel_speed_);
  if (std::abs(target.left) < stop_velocity_threshold_ && std::abs(target.right) < stop_velocity_threshold_) {
    stopMotors();
    return;
  }

  if (now - last_feedback_time_ > feedback_timeout_) {
    RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Encoder feedback stale; motors coasting");
    stopMotors();
    return;
  }

  // The first cycle after a stop has no meaningful dt; PidController then applies feed-forward only.
  const double control_dt = motors_active_ ? dt : 0.0;
  const double left_effort = left_pid_->update(target.left, measured_.left, control_dt);
  const double right_effort = right_pid_->update(target.right, measured_.right, control_dt);
  motors_active_ = true;

  if (!left_motor_->setEffort(left_effort) || !right_motor_->setEffort(right_effort)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Failed to write motor outputs");
  }
}

void MotorDriverNode::stopMotors()
{
  if (left_pid_) {
    left_pid_->reset();
    right_pid_->reset();
  }
  motors_active_ = false;
  if (left_motor_ && right_motor_ && !(left_motor_->coast() && right_motor_->coast())) {
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, kWarnThrottleMs, "Failed to coast motors");
  }
}

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::MotorDriverNode)
