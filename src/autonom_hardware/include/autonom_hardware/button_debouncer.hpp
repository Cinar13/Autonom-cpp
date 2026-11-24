#pragma once

namespace autonom_hardware
{

/// Turns raw button samples into single press events, free of ROS and GPIO so it can be unit tested.
/// A level counts only after `stable_samples` identical samples in a row. A press is reported once, when the
/// stable level goes released -> pressed; the button must first be seen released, so a button held (or shorted)
/// at power-up never produces a press.
class ButtonDebouncer
{
public:
  explicit ButtonDebouncer(int stable_samples);

  /// Feeds one sample; returns true exactly once per debounced press.
  bool update(bool pressed) noexcept;

  [[nodiscard]] bool armed() const noexcept { return armed_; }

private:
  int stable_samples_;
  bool candidate_{false};
  int candidate_count_{0};
  bool stable_pressed_{false};
  bool armed_{false};
};

}  // namespace autonom_hardware
