#include "autonom_hardware/gpio.hpp"

#include <cerrno>
#include <system_error>
#include <utility>

namespace autonom_hardware
{

GpioChipPtr openGpioChip(const std::string & identifier)
{
  gpiod_chip * chip = gpiod_chip_open_lookup(identifier.c_str());
  if (chip == nullptr) {
    throw std::system_error(errno, std::generic_category(), "Failed to open GPIO chip '" + identifier + "'");
  }
  return GpioChipPtr(chip, gpiod_chip_close);
}

GpioOutput::GpioOutput(GpioChipPtr chip, unsigned int offset, const std::string & consumer, bool initial_value)
: chip_(std::move(chip)), line_(gpiod_chip_get_line(chip_.get(), offset)), value_(initial_value)
{
  if (line_ == nullptr) {
    throw std::system_error(errno, std::generic_category(), "GPIO line " + std::to_string(offset) + " not found");
  }
  if (gpiod_line_request_output(line_, consumer.c_str(), initial_value ? 1 : 0) < 0) {
    throw std::system_error(errno, std::generic_category(), "Failed to request GPIO " + std::to_string(offset) + " as output");
  }
}

GpioOutput::~GpioOutput()
{
  gpiod_line_set_value(line_, 0);
  gpiod_line_release(line_);
}

bool GpioOutput::set(bool value) noexcept
{
  if (value == value_) {
    return true;
  }
  if (gpiod_line_set_value(line_, value ? 1 : 0) < 0) {
    return false;
  }
  value_ = value;
  return true;
}

}  // namespace autonom_hardware
