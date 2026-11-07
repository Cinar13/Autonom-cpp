#pragma once

#include <string>

#include "autonom_hardware/gpio.hpp"
#include "autonom_hardware/pwm_channel.hpp"

namespace autonom_hardware
{

struct A4950Config
{
  std::string pwm_chip_path;
  unsigned int pwm_channel{0};
  unsigned int direction_pin{0};
  double pwm_frequency_hz{20000.0};
  bool inverted{false};
};

/// One A4950 H-bridge driven in sign-magnitude mode: IN1 = hardware PWM, IN2 = direction GPIO.
/// With IN2 high the bridge drives in reverse while IN1 is low and brakes while IN1 is high,
/// so the PWM duty is complemented in reverse to keep the commanded effort linear.
class A4950Motor
{
public:
  A4950Motor(const GpioChipPtr & chip, const A4950Config & config, const std::string & consumer);
  ~A4950Motor();

  A4950Motor(const A4950Motor &) = delete;
  A4950Motor & operator=(const A4950Motor &) = delete;

  /// `effort` in [-1, 1]; values outside are clamped. Returns false on a hardware write failure.
  [[nodiscard]] bool setEffort(double effort) noexcept;
  [[nodiscard]] bool coast() noexcept;

private:
  PwmChannel pwm_;
  GpioOutput direction_;
  bool inverted_;
};

}  // namespace autonom_hardware
