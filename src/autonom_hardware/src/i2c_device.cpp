#include "autonom_hardware/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

namespace autonom_hardware
{

I2cDevice::I2cDevice(const std::string & bus_path, std::uint8_t address)
{
  fd_ = ::open(bus_path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "Failed to open " + bus_path);
  }
  if (::ioctl(fd_, I2C_SLAVE, static_cast<unsigned long>(address)) < 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "Failed to select I2C address on " + bus_path);
  }
}

I2cDevice::~I2cDevice()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool I2cDevice::read(std::uint8_t * data, std::size_t length) noexcept
{
  return ::read(fd_, data, length) == static_cast<ssize_t>(length);
}

bool I2cDevice::write(const std::uint8_t * data, std::size_t length) noexcept
{
  return ::write(fd_, data, length) == static_cast<ssize_t>(length);
}

}  // namespace autonom_hardware
