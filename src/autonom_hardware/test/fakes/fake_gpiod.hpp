#pragma once

// Test control surface for fake_gpiod.cpp, an in-process stand-in for the libgpiod v1.6 C API.
// It models one "pinctrl-rp1" chip with 54 lines (like the Pi 5's RP1). Line state lives per offset, shared by
// every chip handle, the way the kernel shares it, so two requests for one pin fail with EBUSY.

#include <cstdint>
#include <string>
#include <vector>

namespace fake_gpiod
{

constexpr unsigned int kNumLines = 54;

enum class Mode { kFree, kOutput, kInput, kEdgeEvents };

struct LineInfo
{
  Mode mode{Mode::kFree};
  std::string consumer;
  bool pull_up{false};
  int value{0};
};

struct Write
{
  unsigned int offset;
  int value;
};

/// Forgets every request, level, queued event and recorded write.
void reset();
/// Lines that are not requested report kFree; the kernel would then drive nothing.
LineInfo line(unsigned int offset);
/// Level an input or edge-event line reads (without this, a pull-up line reads 1, otherwise 0).
void setInputLevel(unsigned int offset, int value);
/// Queues an edge on an edge-event line; also updates the level it reads. `ns` is the kernel timestamp.
void pushEdge(unsigned int offset, bool rising, std::int64_t ns);
struct Edge
{
  unsigned int offset;
  bool rising;
  std::int64_t ns;
};
/// Queues several edges at once, so an event reader sees them all in a single wake-up.
void pushEdges(const std::vector<Edge> & edges);
/// Every successful set_value and the initial value of every output request, in order.
std::vector<Write> writes();
void clearWrites();
/// Makes every following set_value / get_value fail (returns -1, errno EIO) until cleared.
void failIo(bool fail);
/// Makes the next event wait fail with EIO (to simulate a dead event thread).
void failNextWait();
/// Number of chip handles currently open.
int openChips();

}  // namespace fake_gpiod
