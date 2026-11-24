#include "fake_gpiod.hpp"

#include <gpiod.h>

#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>

struct gpiod_line
{
  gpiod_chip * chip;
  unsigned int offset;
};

struct gpiod_chip
{
  std::map<unsigned int, std::unique_ptr<gpiod_line>> lines;
};

namespace fake_gpiod
{
namespace
{

struct KernelLine
{
  LineInfo info;
  bool input_override{false};
  int input_level{0};
  const gpiod_line * owner{nullptr};
  std::deque<gpiod_line_event> events;
};

struct Kernel
{
  std::mutex mutex;
  std::condition_variable events_ready;
  KernelLine lines[kNumLines];
  std::vector<Write> writes;
  bool fail_io{false};
  bool fail_next_wait{false};
  int open_chips{0};
};

Kernel & kernel()
{
  static Kernel instance;
  return instance;
}

int inputLevel(const KernelLine & line)
{
  return line.input_override ? line.input_level : (line.info.pull_up ? 1 : 0);
}

int request(gpiod_line * line, const char * consumer, Mode mode, bool pull_up, int value)
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  KernelLine & state = k.lines[line->offset];
  if (state.owner != nullptr) {
    errno = EBUSY;
    return -1;
  }
  state.owner = line;
  state.info.mode = mode;
  state.info.consumer = consumer != nullptr ? consumer : "";
  state.info.pull_up = pull_up;
  state.events.clear();
  if (mode == Mode::kOutput) {
    state.info.value = value;
    k.writes.push_back({line->offset, value});
  }
  return 0;
}

}  // namespace

void reset()
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  for (auto & line : k.lines) {
    line = KernelLine{};
  }
  k.writes.clear();
  k.fail_io = false;
  k.fail_next_wait = false;
}

LineInfo line(unsigned int offset)
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  return k.lines[offset].info;
}

void setInputLevel(unsigned int offset, int value)
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  k.lines[offset].input_override = true;
  k.lines[offset].input_level = value;
}

void pushEdges(const std::vector<Edge> & edges)
{
  auto & k = kernel();
  {
    std::lock_guard<std::mutex> lock(k.mutex);
    for (const Edge & edge : edges) {
      KernelLine & state = k.lines[edge.offset];
      state.input_override = true;
      state.input_level = edge.rising ? 1 : 0;
      gpiod_line_event event{};
      event.ts.tv_sec = static_cast<time_t>(edge.ns / 1'000'000'000);
      event.ts.tv_nsec = static_cast<long>(edge.ns % 1'000'000'000);
      event.event_type = edge.rising ? GPIOD_LINE_EVENT_RISING_EDGE : GPIOD_LINE_EVENT_FALLING_EDGE;
      state.events.push_back(event);
    }
  }
  k.events_ready.notify_all();
}

void pushEdge(unsigned int offset, bool rising, std::int64_t ns)
{
  pushEdges({{offset, rising, ns}});
}

std::vector<Write> writes()
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  return k.writes;
}

void clearWrites()
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  k.writes.clear();
}

void failIo(bool fail)
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  k.fail_io = fail;
}

void failNextWait()
{
  auto & k = kernel();
  {
    std::lock_guard<std::mutex> lock(k.mutex);
    k.fail_next_wait = true;
  }
  k.events_ready.notify_all();
}

int openChips()
{
  auto & k = kernel();
  std::lock_guard<std::mutex> lock(k.mutex);
  return k.open_chips;
}

}  // namespace fake_gpiod

using fake_gpiod::kernel;

extern "C" {

gpiod_chip * gpiod_chip_open_lookup(const char * descr)
{
  const std::string name = descr != nullptr ? descr : "";
  if (name != "pinctrl-rp1" && name != "gpiochip0" && name != "/dev/gpiochip0" && name != "0") {
    errno = ENOENT;
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(kernel().mutex);
  ++kernel().open_chips;
  return new gpiod_chip;
}

void gpiod_chip_close(gpiod_chip * chip)
{
  {
    std::lock_guard<std::mutex> lock(kernel().mutex);
    for (auto & entry : chip->lines) {
      auto & state = kernel().lines[entry.first];
      if (state.owner == entry.second.get()) {  // closing the chip releases its lines, as libgpiod does
        state.owner = nullptr;
        state.info.mode = fake_gpiod::Mode::kFree;
      }
    }
    --kernel().open_chips;
  }
  delete chip;
}

gpiod_line * gpiod_chip_get_line(gpiod_chip * chip, unsigned int offset)
{
  if (offset >= fake_gpiod::kNumLines) {
    errno = EINVAL;
    return nullptr;
  }
  auto & slot = chip->lines[offset];
  if (!slot) {
    slot = std::make_unique<gpiod_line>(gpiod_line{chip, offset});
  }
  return slot.get();
}

unsigned int gpiod_line_offset(gpiod_line * line)
{
  return line->offset;
}

int gpiod_line_request_output(gpiod_line * line, const char * consumer, int default_val)
{
  return fake_gpiod::request(line, consumer, fake_gpiod::Mode::kOutput, false, default_val ? 1 : 0);
}

int gpiod_line_request_input_flags(gpiod_line * line, const char * consumer, int flags)
{
  return fake_gpiod::request(
    line, consumer, fake_gpiod::Mode::kInput, (flags & GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) != 0, 0);
}

int gpiod_line_request_both_edges_events_flags(gpiod_line * line, const char * consumer, int flags)
{
  return fake_gpiod::request(
    line, consumer, fake_gpiod::Mode::kEdgeEvents, (flags & GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) != 0, 0);
}

void gpiod_line_release(gpiod_line * line)
{
  std::lock_guard<std::mutex> lock(kernel().mutex);
  auto & state = kernel().lines[line->offset];
  if (state.owner == line) {
    state.owner = nullptr;
    state.info.mode = fake_gpiod::Mode::kFree;
    state.events.clear();
  }
}

int gpiod_line_get_value(gpiod_line * line)
{
  std::lock_guard<std::mutex> lock(kernel().mutex);
  auto & state = kernel().lines[line->offset];
  if (kernel().fail_io || state.owner != line) {
    errno = kernel().fail_io ? EIO : EPERM;
    return -1;
  }
  return state.info.mode == fake_gpiod::Mode::kOutput ? state.info.value : fake_gpiod::inputLevel(state);
}

int gpiod_line_set_value(gpiod_line * line, int value)
{
  std::lock_guard<std::mutex> lock(kernel().mutex);
  auto & state = kernel().lines[line->offset];
  if (kernel().fail_io || state.owner != line || state.info.mode != fake_gpiod::Mode::kOutput) {
    errno = kernel().fail_io ? EIO : EPERM;
    return -1;
  }
  state.info.value = value ? 1 : 0;
  kernel().writes.push_back({line->offset, state.info.value});
  return 0;
}

int gpiod_line_event_wait_bulk(gpiod_line_bulk * bulk, const timespec * timeout, gpiod_line_bulk * event_bulk)
{
  auto & k = kernel();
  std::unique_lock<std::mutex> lock(k.mutex);
  const auto ready = [&] {
    if (k.fail_next_wait) {
      return true;
    }
    for (unsigned int i = 0; i < bulk->num_lines; ++i) {
      if (!k.lines[bulk->lines[i]->offset].events.empty()) {
        return true;
      }
    }
    return false;
  };
  const auto wait = std::chrono::seconds(timeout->tv_sec) + std::chrono::nanoseconds(timeout->tv_nsec);
  if (!k.events_ready.wait_for(lock, wait, ready)) {
    return 0;
  }
  if (k.fail_next_wait) {
    k.fail_next_wait = false;
    errno = EIO;
    return -1;
  }
  gpiod_line_bulk_init(event_bulk);
  for (unsigned int i = 0; i < bulk->num_lines; ++i) {
    if (!k.lines[bulk->lines[i]->offset].events.empty()) {
      gpiod_line_bulk_add(event_bulk, bulk->lines[i]);
    }
  }
  return 1;
}

int gpiod_line_event_read_multiple(gpiod_line * line, gpiod_line_event * events, unsigned int num_events)
{
  std::lock_guard<std::mutex> lock(kernel().mutex);
  auto & queue = kernel().lines[line->offset].events;
  unsigned int read = 0;
  while (read < num_events && !queue.empty()) {
    events[read++] = queue.front();
    queue.pop_front();
  }
  return static_cast<int>(read);
}

}  // extern "C"
