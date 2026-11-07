#include "autonom_race/start_gate.hpp"

#include <algorithm>
#include <stdexcept>

namespace autonom_race
{

const char * toString(RaceState state) noexcept
{
  switch (state) {
    case RaceState::kIdle:
      return "IDLE";
    case RaceState::kCountdown:
      return "COUNTDOWN";
    case RaceState::kRacing:
      return "RACING";
    case RaceState::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

StartGate::StartGate(Duration start_delay)
: start_delay_(start_delay)
{
  if (start_delay_ < Duration::zero()) {
    throw std::invalid_argument("start_delay must not be negative");
  }
}

bool StartGate::requestStart(Duration now) noexcept
{
  if (state_ != RaceState::kIdle && state_ != RaceState::kStopped) {
    return false;
  }
  start_time_ = now;
  state_ = RaceState::kCountdown;
  return true;
}

bool StartGate::requestStop() noexcept
{
  if (state_ == RaceState::kIdle || state_ == RaceState::kStopped) {
    return false;
  }
  state_ = RaceState::kStopped;
  return true;
}

RaceState StartGate::update(Duration now) noexcept
{
  if (state_ != RaceState::kCountdown) {
    return state_;
  }
  if (now < start_time_) {
    start_time_ = now;
  }
  if (now - start_time_ >= start_delay_) {
    state_ = RaceState::kRacing;
  }
  return state_;
}

StartGate::Duration StartGate::remaining(Duration now) const noexcept
{
  if (state_ != RaceState::kCountdown) {
    return Duration::zero();
  }
  const auto elapsed = std::max(now - start_time_, Duration::zero());
  return std::max(start_delay_ - elapsed, Duration::zero());
}

}  // namespace autonom_race
