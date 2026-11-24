#pragma once

#include <string>

#include "autonom_hardware/gpio.hpp"
#include "autonom_hardware/pwm_channel.hpp"

namespace autonom_hardware
{

struct Tb6612Config
{
  std::string pwm_chip_path;
  unsigned int pwm_channel{0};
  unsigned int in1_pin{0};
  unsigned int in2_pin{0};
  double pwm_frequency_hz{20000.0};
  bool inverted{false};
};

/// One TB6612FNG channel (A or B): PWMx = hardware PWM, xIN1/xIN2 = direction GPIOs.
/// IN1 high / IN2 low drives forward, IN1 low / IN2 high reverse; PWM low in either case short-brakes, so the
/// duty cycle is the effort magnitude in both directions. IN1 = IN2 = low with duty 0 coasts (outputs off).
/// The chip-wide STBY pin is not owned here: the caller holds it (see MotorDriverNode).
class Tb6612Motor
{
public:
  Tb6612Motor(const GpioChipPtr & chip, const Tb6612Config & config, const std::string & consumer);
  ~Tb6612Motor();

  Tb6612Motor(const Tb6612Motor &) = delete;
  Tb6612Motor & operator=(const Tb6612Motor &) = delete;

  /// `effort` in [-1, 1]; values outside are clamped. Returns false on a hardware write failure.
  [[nodiscard]] bool setEffort(double effort) noexcept;
  [[nodiscard]] bool coast() noexcept;

private:
  // Direction pins first: both are driven low before the PWM channel is claimed, so pins left high by a crashed
  // process never combine with a live duty cycle. Destruction then disables the PWM before releasing them.
  GpioOutput in1_;
  GpioOutput in2_;
  PwmChannel pwm_;
  bool inverted_;
};

}  // namespace autonom_hardware
