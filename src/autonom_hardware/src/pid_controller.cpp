#include "autonom_hardware/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autonom_hardware
{

PidController::PidController(const PidGains & gains)
: gains_(gains)
{
  validate(gains_);
}

void PidController::validate(const PidGains & gains)
{
  for (const double value : {gains.kp, gains.ki, gains.kd, gains.kf, gains.integral_limit, gains.output_limit}) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("PID gains and limits must be finite and non-negative");
    }
  }
  if (gains.output_limit == 0.0) {
    throw std::invalid_argument("PID output_limit must be positive");
  }
}

void PidController::setGains(const PidGains & gains)
{
  validate(gains);
  gains_ = gains;
  integral_ = std::clamp(integral_, -gains_.integral_limit, gains_.integral_limit);
}

void PidController::reset() noexcept
{
  integral_ = 0.0;
  has_previous_measurement_ = false;
  previous_setpoint_sign_ = 0;
}

double PidController::update(double setpoint, double measurement, double dt) noexcept
{
  const double feed_forward = gains_.kf * setpoint;
  if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
    reset();
    return 0.0;
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return std::clamp(feed_forward, -gains_.output_limit, gains_.output_limit);
  }

  // A direction change makes the wound-up integral push the wrong way: on the robot the wheels then overshot
  // and reversed again and again ("motors go back and forth really fast") while Nav2 braked and re-accelerated.
  const int setpoint_sign = (setpoint > 0.0) - (setpoint < 0.0);
  if (setpoint_sign != previous_setpoint_sign_) {
    integral_ = 0.0;
    previous_setpoint_sign_ = setpoint_sign;
  }

  const double error = setpoint - measurement;
  const double derivative = has_previous_measurement_ ? -(measurement - previous_measurement_) / dt : 0.0;
  previous_measurement_ = measurement;
  has_previous_measurement_ = true;

  const double candidate_integral =
    std::clamp(integral_ + gains_.ki * error * dt, -gains_.integral_limit, gains_.integral_limit);
  const double unsaturated = feed_forward + gains_.kp * error + candidate_integral + gains_.kd * derivative;
  const double output = std::clamp(unsaturated, -gains_.output_limit, gains_.output_limit);

  // Only integrate when not saturated, or when integrating drives the output back out of saturation.
  if (output == unsaturated || (unsaturated > output) != (error > 0.0)) {
    integral_ = candidate_integral;
  }
  return output;
}

}  // namespace autonom_hardware
