#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "autonom_hardware/i2c_device.hpp"

namespace autonom_hardware
{

/// CEVA BNO08x (BNO080/085/086) over I2C: the SHTP transport and the few SH-2 messages this robot needs.
/// No CEVA library: the full SH-2 stack waits for an SHTP advertisement that the robot's sensor never sends.
/// References: CEVA "SH-2 Reference Manual" (1000-3625) and "SH-2 SHTP Reference Manual" (1000-3535).
namespace bno08x
{

/// SHTP channels used by the SH-2 application.
enum Channel : std::uint8_t
{
  kCommandChannel = 0,
  kExecutableChannel = 1,
  kControlChannel = 2,
  kInputReportChannel = 3,
  kWakeReportChannel = 4,
  kGyroRotationVectorChannel = 5,
};

/// Sensor report IDs (SH-2 manual 6.5).
enum ReportId : std::uint8_t
{
  kAccelerometer = 0x01,          // m/s^2, gravity included, Q8
  kGyroscopeCalibrated = 0x02,    // rad/s, Q9
  kMagneticFieldCalibrated = 0x03,  // uT, Q4
  kLinearAcceleration = 0x04,     // m/s^2, gravity removed, Q8
  kRotationVector = 0x05,         // quaternion, Q14 (magnetometer referenced)
  kGravity = 0x06,                // m/s^2, Q8
  kGameRotationVector = 0x08,     // quaternion, Q14 (no magnetometer: yaw drifts, immune to motor fields)
};

constexpr std::size_t kHeaderSize = 4;
constexpr std::size_t kMaxPacketSize = 1024;

struct Packet
{
  std::uint8_t channel{0};
  std::uint8_t sequence{0};
  std::vector<std::uint8_t> payload;
};

struct SensorEvent
{
  std::uint8_t report_id{0};
  std::uint8_t accuracy{0};  // 0 unreliable .. 3 high
  std::array<double, 3> vector{};  // x, y, z for the 3-axis reports
  std::array<double, 4> quaternion{0.0, 0.0, 0.0, 1.0};  // x, y, z, w for the rotation vectors
};

struct ProductId
{
  std::uint8_t reset_cause{0};
  std::uint8_t version_major{0};
  std::uint8_t version_minor{0};
  std::uint16_t version_patch{0};
  std::uint32_t part_number{0};
  std::uint32_t build_number{0};
};

/// Set Feature command (0xFD) enabling `report_id` every `interval_us` microseconds (0 disables it).
std::vector<std::uint8_t> setFeatureCommand(std::uint8_t report_id, std::uint32_t interval_us);

/// Parses an input-report payload (channels 3 and 4): timestamp records followed by sensor reports.
/// Appends every report it understands to `events`; returns false at the first unknown or truncated record.
bool parseInputReports(const std::vector<std::uint8_t> & payload, std::vector<SensorEvent> & events);

/// Product ID Response (0xF8) on the control channel.
std::optional<ProductId> parseProductId(const Packet & packet);

/// Get Feature Response (0xFC) on the control channel: the report it describes and its granted interval [us].
struct FeatureResponse
{
  std::uint8_t report_id{0};
  std::uint32_t interval_us{0};
};
std::optional<FeatureResponse> parseFeatureResponse(const Packet & packet);

/// True for the messages a BNO08x sends after any reset: "reset complete" on the executable channel, or the
/// unsolicited Initialize response (0xF1, command 0x84) on the control channel. Enabled reports are then lost.
bool isResetNotification(const Packet & packet);

}  // namespace bno08x

class Bno08x
{
public:
  enum class ReadResult { kPacket, kEmpty, kError };

  explicit Bno08x(I2cTransport & transport);

  /// Reads one SHTP packet: a 4-byte header read, then one read of the whole packet, which the sensor sends again
  /// behind a continuation header. kEmpty when the sensor has nothing queued.
  ReadResult readPacket(bno08x::Packet & packet);

  [[nodiscard]] bool sendPacket(std::uint8_t channel, const std::vector<std::uint8_t> & payload);

  /// Time the hub needs to reboot after a soft reset before its "reset complete" can be trusted.
  static constexpr std::chrono::milliseconds kResetSettle{300};

  /// Soft reset (executable channel): drains the queue, resets, waits kResetSettle, then for this reset's
  /// "reset complete", then drains the rest of the reset's messages. Drops everything read meanwhile.
  [[nodiscard]] bool softReset(std::chrono::milliseconds timeout);

  /// Product ID request (0xF9); returns the first response entry.
  std::optional<bno08x::ProductId> requestProductId(std::chrono::milliseconds timeout);

  /// Sends Set Feature for one report. The robot's sensor (firmware 2.6.6) starts reporting but sends no Get Feature
  /// Response, so success is judged by the reports arriving, not by a reply.
  [[nodiscard]] bool enableReport(std::uint8_t report_id, std::uint32_t interval_us);

private:
  /// Reads and drops packets until the sensor has nothing queued (three empty reads) or `limit` passes.
  void drain(std::chrono::milliseconds limit);

  template<typename Predicate>
  bool waitForPacket(Predicate && matches, std::chrono::milliseconds timeout, bno08x::Packet & packet);

  I2cTransport & transport_;
  std::array<std::uint8_t, 6> sequence_{};
};

}  // namespace autonom_hardware
