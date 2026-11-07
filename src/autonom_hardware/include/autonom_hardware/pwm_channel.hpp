#pragma once

#include <cstdint>
#include <string>

namespace autonom_hardware
{

/// Hardware PWM channel through the Linux sysfs PWM interface (e.g. RPi 5 RP1 PWM0 via dtoverlay=pwm-2chan).
/// Exports and enables the channel on construction; disables and unexports it on destruction.
class PwmChannel
{
public:
  PwmChannel(const std::string & chip_path, unsigned int channel, double frequency_hz);
  ~PwmChannel();

  PwmChannel(const PwmChannel &) = delete;
  PwmChannel & operator=(const PwmChannel &) = delete;

  /// Non-blocking duty update; `fraction` is clamped to [0, 1].
  [[nodiscard]] bool setDutyCycle(double fraction) noexcept;

private:
  std::string chip_path_;
  std::string channel_path_;
  unsigned int channel_;
  bool exported_by_us_{false};
  int duty_fd_{-1};
  std::uint64_t period_ns_{0};
  std::uint64_t duty_ns_{0};
};

}  // namespace autonom_hardware
