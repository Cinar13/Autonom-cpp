#pragma once

#include <chrono>

namespace autonom_race
{

enum class RaceState
{
  kIdle,       ///< Powered up, waiting for the operator's start signal.
  kCountdown,  ///< Start received; motion stays blocked until the start delay has elapsed (AC-3.2, AC-4.5).
  kRacing,     ///< Motion and Nav2 goals allowed.
  kStopped,    ///< Stop received; motion blocked until the next start signal.
};

const char * toString(RaceState state) noexcept;

/// Competition start sequence, free of ROS so it can be unit tested against any clock.
class StartGate
{
public:
  using Duration = std::chrono::nanoseconds;

  /// Throws std::invalid_argument when `start_delay` is negative.
  explicit StartGate(Duration start_delay);

  /// Begins the countdown from Idle or Stopped. Returns false (and changes nothing) in any other state,
  /// so a bouncing button or a repeated IR code can never restart a running countdown.
  bool requestStart(Duration now) noexcept;

  /// Blocks motion from any state except Idle. Returns true when the state changed.
  bool requestStop() noexcept;

  /// Advances Countdown -> Racing once `now - start >= start_delay`. A clock that jumps backwards
  /// (simulation reset) restarts the countdown from `now`, so the delay is never shortened.
  RaceState update(Duration now) noexcept;

  [[nodiscard]] RaceState state() const noexcept { return state_; }
  [[nodiscard]] bool motionAllowed() const noexcept { return state_ == RaceState::kRacing; }
  /// Time left in the countdown; zero outside Countdown.
  [[nodiscard]] Duration remaining(Duration now) const noexcept;

private:
  Duration start_delay_;
  Duration start_time_{0};
  RaceState state_{RaceState::kIdle};
};

}  // namespace autonom_race
