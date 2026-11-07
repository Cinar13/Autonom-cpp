#include "autonom_hardware/pwm_channel.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace autonom_hardware
{
namespace
{

constexpr auto kSysfsReadyTimeout = std::chrono::seconds(2);
constexpr auto kSysfsPollInterval = std::chrono::milliseconds(10);

bool writeAttribute(const std::string & path, const std::string & value) noexcept
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  const auto written = ::write(fd, value.data(), value.size());
  ::close(fd);
  return written == static_cast<ssize_t>(value.size());
}

void writeAttributeOrThrow(const std::string & path, const std::string & value)
{
  if (!writeAttribute(path, value)) {
    throw std::system_error(errno, std::generic_category(), "Failed to write '" + value + "' to " + path);
  }
}

// After export, udev may need a moment to create the attributes and apply group permissions.
void waitUntilWritable(const std::string & path)
{
  const auto deadline = std::chrono::steady_clock::now() + kSysfsReadyTimeout;
  while (::access(path.c_str(), W_OK) != 0) {
    if (std::chrono::steady_clock::now() > deadline) {
      throw std::runtime_error("Timed out waiting for writable " + path + " (is the user in the gpio group?)");
    }
    std::this_thread::sleep_for(kSysfsPollInterval);
  }
}

}  // namespace

PwmChannel::PwmChannel(const std::string & chip_path, unsigned int channel, double frequency_hz)
: chip_path_(chip_path), channel_path_(chip_path + "/pwm" + std::to_string(channel)), channel_(channel)
{
  if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
    throw std::invalid_argument("PWM frequency must be positive");
  }
  period_ns_ = static_cast<std::uint64_t>(std::llround(1e9 / frequency_hz));
  if (period_ns_ == 0) {
    throw std::invalid_argument("PWM frequency too high");
  }

  if (!std::filesystem::exists(channel_path_)) {
    writeAttributeOrThrow(chip_path_ + "/export", std::to_string(channel_));
    exported_by_us_ = true;
  }
  waitUntilWritable(channel_path_ + "/period");
  waitUntilWritable(channel_path_ + "/duty_cycle");

  // duty_cycle must never exceed period, so zero it before changing the period.
  writeAttribute(channel_path_ + "/duty_cycle", "0");
  writeAttributeOrThrow(channel_path_ + "/period", std::to_string(period_ns_));
  writeAttributeOrThrow(channel_path_ + "/enable", "1");

  duty_fd_ = ::open((channel_path_ + "/duty_cycle").c_str(), O_WRONLY | O_CLOEXEC);
  if (duty_fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "Failed to open " + channel_path_ + "/duty_cycle");
  }
}

PwmChannel::~PwmChannel()
{
  if (duty_fd_ >= 0) {
    static_cast<void>(setDutyCycle(0.0));
    ::close(duty_fd_);
  }
  writeAttribute(channel_path_ + "/enable", "0");
  if (exported_by_us_) {
    writeAttribute(chip_path_ + "/unexport", std::to_string(channel_));
  }
}

bool PwmChannel::setDutyCycle(double fraction) noexcept
{
  if (!std::isfinite(fraction)) {
    fraction = 0.0;
  }
  const auto duty_ns = static_cast<std::uint64_t>(std::llround(std::clamp(fraction, 0.0, 1.0) * static_cast<double>(period_ns_)));
  if (duty_ns == duty_ns_) {
    return true;
  }
  char buffer[24];
  const int length = std::snprintf(buffer, sizeof(buffer), "%llu", static_cast<unsigned long long>(duty_ns));
  if (length <= 0 || ::pwrite(duty_fd_, buffer, static_cast<size_t>(length), 0) != length) {
    return false;
  }
  duty_ns_ = duty_ns;
  return true;
}

}  // namespace autonom_hardware
