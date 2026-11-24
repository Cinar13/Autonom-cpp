#include "autonom_hardware/button_debouncer.hpp"

#include <stdexcept>

namespace autonom_hardware
{

ButtonDebouncer::ButtonDebouncer(int stable_samples)
: stable_samples_(stable_samples)
{
  if (stable_samples < 1) {
    throw std::invalid_argument("stable_samples must be at least 1");
  }
}

bool ButtonDebouncer::update(bool pressed) noexcept
{
  if (pressed != candidate_) {
    candidate_ = pressed;
    candidate_count_ = 0;
  }
  if (candidate_count_ < stable_samples_) {
    ++candidate_count_;
  }
  if (candidate_count_ < stable_samples_ || (armed_ && candidate_ == stable_pressed_)) {
    return false;
  }
  if (!armed_) {
    // First stable level: only a released button arms it.
    stable_pressed_ = candidate_;
    armed_ = !candidate_;
    return false;
  }
  stable_pressed_ = candidate_;
  return stable_pressed_;
}

}  // namespace autonom_hardware
