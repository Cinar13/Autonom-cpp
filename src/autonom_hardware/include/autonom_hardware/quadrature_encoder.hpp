#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "autonom_hardware/gpio.hpp"

namespace autonom_hardware
{

struct QuadratureEncoderConfig
{
  unsigned int pin_a{0};
  unsigned int pin_b{0};
  bool inverted{false};
  bool pull_up{true};
};

/// x4 quadrature decoder fed by kernel edge interrupts (libgpiod line events).
/// A dedicated worker thread drains events; readers only touch lock-free atomics.
class QuadratureEncoder
{
public:
  QuadratureEncoder(const GpioChipPtr & chip, const QuadratureEncoderConfig & config, const std::string & consumer);
  ~QuadratureEncoder();

  QuadratureEncoder(const QuadratureEncoder &) = delete;
  QuadratureEncoder & operator=(const QuadratureEncoder &) = delete;

  std::int64_t count() const noexcept { return count_.load(std::memory_order_relaxed); }
  /// Edges whose level matched the tracked level, i.e. the opposite edge was lost (noise or overload).
  std::uint64_t missedTransitions() const noexcept { return missed_.load(std::memory_order_relaxed); }
  bool healthy() const noexcept { return healthy_.load(std::memory_order_relaxed); }

private:
  void requestLine(gpiod_line * line, const std::string & consumer, bool pull_up);
  void eventLoop() noexcept;
  void applyEdge(bool is_channel_a, bool level) noexcept;

  GpioChipPtr chip_;
  gpiod_line * line_a_{nullptr};
  gpiod_line * line_b_{nullptr};
  std::int64_t direction_sign_;
  std::uint8_t state_{0};
  std::atomic<std::int64_t> count_{0};
  std::atomic<std::uint64_t> missed_{0};
  std::atomic<bool> healthy_{true};
  std::atomic<bool> running_{true};
  std::thread worker_;
};

}  // namespace autonom_hardware
