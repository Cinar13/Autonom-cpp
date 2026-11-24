#include "autonom_hardware/tb6612_motor.hpp"

#include <algorithm>
#include <cmath>

namespace autonom_hardware
{

Tb6612Motor::Tb6612Motor(const GpioChipPtr & chip, const Tb6612Config & config, const std::string & consumer)
: in1_(chip, config.in1_pin, consumer + "_in1", false),
  in2_(chip, config.in2_pin, consumer + "_in2", false),
  pwm_(config.pwm_chip_path, config.pwm_channel, config.pwm_frequency_hz),
  inverted_(config.inverted)
{
}

Tb6612Motor::~Tb6612Motor()
{
  static_cast<void>(coast());
}

bool Tb6612Motor::setEffort(double effort) noexcept
{
  if (!std::isfinite(effort) || effort == 0.0) {
    return coast();
  }
  effort = std::clamp(inverted_ ? -effort : effort, -1.0, 1.0);
  const bool reverse = effort < 0.0;
  // Lower the pin being released before raising the other: IN1 = IN2 = high is a short brake, never a glitch
  // through both directions.
  bool ok = reverse ? in1_.set(false) : in2_.set(false);
  ok = (reverse ? in2_.set(true) : in1_.set(true)) && ok;
  ok = pwm_.setDutyCycle(std::abs(effort)) && ok;
  return ok;
}

bool Tb6612Motor::coast() noexcept
{
  const bool pwm_ok = pwm_.setDutyCycle(0.0);
  const bool in1_ok = in1_.set(false);
  const bool in2_ok = in2_.set(false);
  return pwm_ok && in1_ok && in2_ok;
}

}  // namespace autonom_hardware
