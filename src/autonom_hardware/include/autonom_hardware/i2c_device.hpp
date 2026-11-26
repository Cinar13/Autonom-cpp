#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace autonom_hardware
{

/// Byte-stream access to one device: each call is one complete I2C read or write transaction.
class I2cTransport
{
public:
  virtual ~I2cTransport() = default;
  [[nodiscard]] virtual bool read(std::uint8_t * data, std::size_t length) noexcept = 0;
  [[nodiscard]] virtual bool write(const std::uint8_t * data, std::size_t length) noexcept = 0;
};

/// Linux i2c-dev device (e.g. /dev/i2c-1) bound to one 7-bit address. Throws std::system_error on open failure.
class I2cDevice : public I2cTransport
{
public:
  I2cDevice(const std::string & bus_path, std::uint8_t address);
  ~I2cDevice() override;

  I2cDevice(const I2cDevice &) = delete;
  I2cDevice & operator=(const I2cDevice &) = delete;

  [[nodiscard]] bool read(std::uint8_t * data, std::size_t length) noexcept override;
  [[nodiscard]] bool write(const std::uint8_t * data, std::size_t length) noexcept override;

private:
  int fd_{-1};
};

}  // namespace autonom_hardware
