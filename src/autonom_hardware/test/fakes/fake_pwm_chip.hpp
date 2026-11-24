#pragma once

// A temporary directory laid out like /sys/class/pwm/pwmchipN, for PwmChannel tests.
// With `auto_export`, a watcher thread plays the kernel: a channel number written to `export` creates pwmN/,
// one written to `unexport` removes it.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

namespace fake_pwm
{

class FakePwmChip
{
public:
  explicit FakePwmChip(bool auto_export = true)
  {
    std::string pattern = (std::filesystem::temp_directory_path() / "fake_pwmchip_XXXXXX").string();
    if (::mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = pattern;
    touch(path_ + "/export");
    touch(path_ + "/unexport");
    if (auto_export) {
      watcher_ = std::thread([this] { watch(); });
    }
  }

  ~FakePwmChip()
  {
    running_ = false;
    if (watcher_.joinable()) {
      watcher_.join();
    }
    std::filesystem::remove_all(path_);
  }

  const std::string & path() const { return path_; }

  /// Creates pwmN/ as if it was already exported (by an earlier process).
  void precreate(unsigned int channel) const { createChannel(channel); }
  bool exists(unsigned int channel) const { return std::filesystem::exists(channelPath(channel)); }
  /// The watcher unexports asynchronously (the kernel does it synchronously): wait for it.
  bool waitUntilGone(unsigned int channel) const
  {
    for (int i = 0; i < 500 && exists(channel); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !exists(channel);
  }

  /// The value most recently written to pwmN/<attribute> (up to the first newline), or "" if absent/empty.
  std::string read(unsigned int channel, const std::string & attribute) const
  {
    std::ifstream file(channelPath(channel) + "/" + attribute);
    std::string value;
    std::getline(file, value);
    return value;
  }
  long long readNumber(unsigned int channel, const std::string & attribute) const
  {
    const std::string value = read(channel, attribute);
    return value.empty() ? -1 : std::stoll(value);
  }

private:
  std::string channelPath(unsigned int channel) const { return path_ + "/pwm" + std::to_string(channel); }

  static void touch(const std::string & file) { std::ofstream(file).close(); }

  void createChannel(unsigned int channel) const
  {
    std::filesystem::create_directories(channelPath(channel));
    for (const char * attribute : {"period", "duty_cycle", "enable"}) {
      touch(channelPath(channel) + "/" + attribute);
    }
  }

  // Returns and clears the number written to a control file, or -1.
  int take(const std::string & file) const
  {
    std::ifstream in(file);
    int value = -1;
    if (!(in >> value)) {
      return -1;
    }
    in.close();
    touch(file);
    return value;
  }

  void watch()
  {
    while (running_) {
      const int exported = take(path_ + "/export");
      if (exported >= 0) {
        createChannel(static_cast<unsigned int>(exported));
      }
      const int unexported = take(path_ + "/unexport");
      if (unexported >= 0) {
        std::filesystem::remove_all(channelPath(static_cast<unsigned int>(unexported)));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::string path_;
  std::atomic<bool> running_{true};
  std::thread watcher_;
};

}  // namespace fake_pwm
