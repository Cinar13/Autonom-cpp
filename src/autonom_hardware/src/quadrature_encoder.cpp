#include "autonom_hardware/quadrature_encoder.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace autonom_hardware
{
namespace
{

constexpr timespec kEventWaitTimeout{0, 100'000'000};
constexpr unsigned int kMaxEventsPerRead = 16;

// Indexed by (previous_state << 2) | current_state, state = (A << 1) | B.
constexpr std::array<std::int8_t, 16> kTransitionTable{
  0, +1, -1, 0,
  -1, 0, 0, +1,
  +1, 0, 0, -1,
  0, -1, +1, 0,
};

struct TimedEdge
{
  timespec timestamp;
  bool is_channel_a;
  bool level;
};

bool earlier(const TimedEdge & lhs, const TimedEdge & rhs) noexcept
{
  return lhs.timestamp.tv_sec != rhs.timestamp.tv_sec ? lhs.timestamp.tv_sec < rhs.timestamp.tv_sec :
                                                        lhs.timestamp.tv_nsec < rhs.timestamp.tv_nsec;
}

}  // namespace

QuadratureEncoder::QuadratureEncoder(
  const GpioChipPtr & chip, const QuadratureEncoderConfig & config, const std::string & consumer)
: chip_(chip), direction_sign_(config.inverted ? -1 : 1)
{
  if (config.pin_a == config.pin_b) {
    throw std::invalid_argument("Encoder channels A and B must use different GPIO pins");
  }
  line_a_ = gpiod_chip_get_line(chip_.get(), config.pin_a);
  line_b_ = gpiod_chip_get_line(chip_.get(), config.pin_b);
  if (line_a_ == nullptr || line_b_ == nullptr) {
    throw std::system_error(errno, std::generic_category(), "Encoder GPIO line not found");
  }

  requestLine(line_a_, consumer, config.pull_up);
  try {
    requestLine(line_b_, consumer, config.pull_up);
  } catch (...) {
    gpiod_line_release(line_a_);
    throw;
  }

  const int level_a = gpiod_line_get_value(line_a_);
  const int level_b = gpiod_line_get_value(line_b_);
  if (level_a < 0 || level_b < 0) {
    gpiod_line_release(line_a_);
    gpiod_line_release(line_b_);
    throw std::system_error(errno, std::generic_category(), "Failed to read initial encoder state");
  }
  state_ = static_cast<std::uint8_t>((level_a << 1) | level_b);

  worker_ = std::thread(&QuadratureEncoder::eventLoop, this);
}

QuadratureEncoder::~QuadratureEncoder()
{
  running_.store(false);
  if (worker_.joinable()) {
    worker_.join();
  }
  gpiod_line_release(line_a_);
  gpiod_line_release(line_b_);
}

void QuadratureEncoder::requestLine(gpiod_line * line, const std::string & consumer, bool pull_up)
{
  const int flags = pull_up ? GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP : GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
  if (gpiod_line_request_both_edges_events_flags(line, consumer.c_str(), flags) < 0) {
    throw std::system_error(
      errno, std::generic_category(), "Failed to request edge events on GPIO " + std::to_string(gpiod_line_offset(line)));
  }
}

void QuadratureEncoder::eventLoop() noexcept
{
  gpiod_line_bulk lines;
  gpiod_line_bulk_init(&lines);
  gpiod_line_bulk_add(&lines, line_a_);
  gpiod_line_bulk_add(&lines, line_b_);

  std::array<gpiod_line_event, kMaxEventsPerRead> raw_events{};
  std::array<TimedEdge, 2 * kMaxEventsPerRead> edges{};

  while (running_.load(std::memory_order_relaxed)) {
    gpiod_line_bulk ready;
    const int wait_result = gpiod_line_event_wait_bulk(&lines, &kEventWaitTimeout, &ready);
    if (wait_result == 0) {
      continue;
    }
    if (wait_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      healthy_.store(false);
      return;
    }

    std::size_t edge_count = 0;
    for (unsigned int i = 0; i < gpiod_line_bulk_num_lines(&ready); ++i) {
      gpiod_line * line = gpiod_line_bulk_get_line(&ready, i);
      const int read = gpiod_line_event_read_multiple(line, raw_events.data(), kMaxEventsPerRead);
      if (read < 0) {
        healthy_.store(false);
        return;
      }
      for (int e = 0; e < read; ++e) {
        edges[edge_count++] = {
          raw_events[e].ts, line == line_a_, raw_events[e].event_type == GPIOD_LINE_EVENT_RISING_EDGE};
      }
    }

    // A and B are read separately; restore the true inter-channel order before decoding.
    std::sort(edges.begin(), edges.begin() + static_cast<std::ptrdiff_t>(edge_count), earlier);
    for (std::size_t e = 0; e < edge_count; ++e) {
      applyEdge(edges[e].is_channel_a, edges[e].level);
    }
  }
}

void QuadratureEncoder::applyEdge(bool is_channel_a, bool level) noexcept
{
  const std::uint8_t mask = is_channel_a ? 0b10 : 0b01;
  const auto next_state = static_cast<std::uint8_t>(level ? (state_ | mask) : (state_ & ~mask));
  if (next_state == state_) {
    missed_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const std::int8_t step = kTransitionTable[static_cast<std::size_t>((state_ << 2) | next_state)];
  state_ = next_state;
  count_.fetch_add(direction_sign_ * step, std::memory_order_relaxed);
}

}  // namespace autonom_hardware
