#pragma once

#include <gpiod.h>

#include <memory>
#include <string>

namespace autonom_hardware
{

using GpioChipPtr = std::shared_ptr<gpiod_chip>;

/// Opens a GPIO chip by name, label (e.g. "pinctrl-rp1"), path or number.
GpioChipPtr openGpioChip(const std::string & identifier);

class GpioOutput
{
public:
  GpioOutput(GpioChipPtr chip, unsigned int offset, const std::string & consumer, bool initial_value);
  ~GpioOutput();

  GpioOutput(const GpioOutput &) = delete;
  GpioOutput & operator=(const GpioOutput &) = delete;

  [[nodiscard]] bool set(bool value) noexcept;

private:
  GpioChipPtr chip_;
  gpiod_line * line_;
  bool value_;
};

/// Input line, optionally with the internal pull-up (for a switch to GND: pressed reads low).
class GpioInput
{
public:
  GpioInput(GpioChipPtr chip, unsigned int offset, const std::string & consumer, bool pull_up);
  ~GpioInput();

  GpioInput(const GpioInput &) = delete;
  GpioInput & operator=(const GpioInput &) = delete;

  /// 0 or 1, or -1 on a read failure.
  [[nodiscard]] int read() noexcept;

private:
  GpioChipPtr chip_;
  gpiod_line * line_;
};

}  // namespace autonom_hardware
