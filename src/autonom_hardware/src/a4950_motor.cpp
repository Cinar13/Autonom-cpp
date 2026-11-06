#include "autonom_hardware/a4950_motor.hpp"

#include <algorithm>
#include <cmath>

namespace autonom_hardware
{

A4950Motor::A4950Motor(const GpioChipPtr & chip, const A4950Config & config, const std::string & consumer)
: pwm_(config.pwm_chip_path, config.pwm_channel, config.pwm_frequency_hz),
  direction_(chip, config.direction_pin, consumer, false),
  inverted_(config.inverted)
{
}

A4950Motor::~A4950Motor()
{
  static_cast<void>(coast());
}

bool A4950Motor::setEffort(double effort) noexcept
{
  if (!std::isfinite(effort) || effort == 0.0) {
    return coast();
  }
  effort = std::clamp(inverted_ ? -effort : effort, -1.0, 1.0);
  const bool reverse = effort < 0.0;
  const double magnitude = std::abs(effort);
  const bool pwm_ok = pwm_.setDutyCycle(reverse ? 1.0 - magnitude : magnitude);
  const bool dir_ok = direction_.set(reverse);
  return pwm_ok && dir_ok;
}

bool A4950Motor::coast() noexcept
{
  const bool pwm_ok = pwm_.setDutyCycle(0.0);
  const bool dir_ok = direction_.set(false);
  return pwm_ok && dir_ok;
}

}  // namespace autonom_hardware
