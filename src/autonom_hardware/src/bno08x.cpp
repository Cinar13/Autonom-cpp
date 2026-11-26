#include "autonom_hardware/bno08x.hpp"

#include <thread>

namespace autonom_hardware
{
namespace bno08x
{
namespace
{

constexpr std::uint8_t kSetFeatureCommand = 0xFD;
constexpr std::uint8_t kGetFeatureResponse = 0xFC;
constexpr std::uint8_t kProductIdRequest = 0xF9;
constexpr std::uint8_t kProductIdResponse = 0xF8;
constexpr std::uint8_t kCommandResponse = 0xF1;
constexpr std::uint8_t kInitializeUnsolicited = 0x84;
constexpr std::uint8_t kBaseTimestamp = 0xFB;
constexpr std::uint8_t kTimestampRebase = 0xFA;
constexpr std::uint8_t kExecutableReset = 0x01;
constexpr std::uint8_t kExecutableResetComplete = 0x01;

std::uint16_t u16(const std::uint8_t * p) {return static_cast<std::uint16_t>(p[0] | (p[1] << 8));}
std::int16_t i16(const std::uint8_t * p) {return static_cast<std::int16_t>(u16(p));}
std::uint32_t u32(const std::uint8_t * p)
{
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
void putU32(std::vector<std::uint8_t> & out, std::uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

/// Record length (report ID included) for the IDs this driver can skip or decode; 0 if unknown.
std::size_t reportLength(std::uint8_t id)
{
  switch (id) {
    case kBaseTimestamp:
    case kTimestampRebase:
      return 5;
    case kAccelerometer:
    case kGyroscopeCalibrated:
    case kMagneticFieldCalibrated:
    case kLinearAcceleration:
    case kGravity:
      return 10;
    case kGameRotationVector:
    case 0x29:  // ARVR-stabilised game rotation vector
      return 12;
    case kRotationVector:
    case 0x09:  // geomagnetic rotation vector
    case 0x28:  // ARVR-stabilised rotation vector
      return 14;
    case 0x07:  // gyroscope uncalibrated
    case 0x0F:  // magnetic field uncalibrated
      return 16;
    default:
      return 0;
  }
}

/// Q-point scale of the 3-axis reports.
double vectorScale(std::uint8_t id)
{
  switch (id) {
    case kGyroscopeCalibrated:
      return 1.0 / 512.0;  // Q9
    case kMagneticFieldCalibrated:
      return 1.0 / 16.0;  // Q4
    default:
      return 1.0 / 256.0;  // Q8: accelerometer, linear acceleration, gravity
  }
}

}  // namespace

std::vector<std::uint8_t> setFeatureCommand(std::uint8_t report_id, std::uint32_t interval_us)
{
  std::vector<std::uint8_t> command{kSetFeatureCommand, report_id, 0x00, 0x00, 0x00};  // flags, sensitivity
  putU32(command, interval_us);
  putU32(command, 0);  // batch interval: report immediately
  putU32(command, 0);  // sensor-specific configuration
  return command;
}

bool parseInputReports(const std::vector<std::uint8_t> & payload, std::vector<SensorEvent> & events)
{
  std::size_t pos = 0;
  while (pos < payload.size()) {
    const std::uint8_t id = payload[pos];
    const std::size_t length = reportLength(id);
    if (length == 0 || pos + length > payload.size()) {
      return false;
    }
    const std::uint8_t * record = payload.data() + pos;
    if (id != kBaseTimestamp && id != kTimestampRebase) {
      SensorEvent event;
      event.report_id = id;
      event.accuracy = static_cast<std::uint8_t>(record[2] & 0x03U);
      if (length == 10) {
        const double scale = vectorScale(id);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          event.vector[axis] = i16(record + 4 + 2 * axis) * scale;
        }
        events.push_back(event);
      } else if (id == kGameRotationVector || id == kRotationVector) {
        constexpr double kQ14 = 1.0 / 16384.0;
        for (std::size_t i = 0; i < 4; ++i) {  // i, j, k, real
          event.quaternion[i] = i16(record + 4 + 2 * i) * kQ14;
        }
        events.push_back(event);
      }
    }
    pos += length;
  }
  return true;
}

std::optional<ProductId> parseProductId(const Packet & packet)
{
  const auto & p = packet.payload;
  if (packet.channel != kControlChannel || p.size() < 16 || p[0] != kProductIdResponse) {
    return std::nullopt;
  }
  ProductId id;
  id.reset_cause = p[1];
  id.version_major = p[2];
  id.version_minor = p[3];
  id.part_number = u32(&p[4]);
  id.build_number = u32(&p[8]);
  id.version_patch = u16(&p[12]);
  return id;
}

std::optional<FeatureResponse> parseFeatureResponse(const Packet & packet)
{
  const auto & p = packet.payload;
  if (packet.channel != kControlChannel || p.size() < 9 || p[0] != kGetFeatureResponse) {
    return std::nullopt;
  }
  return FeatureResponse{p[1], u32(&p[5])};
}

bool isResetNotification(const Packet & packet)
{
  const auto & p = packet.payload;
  if (packet.channel == kExecutableChannel) {
    return !p.empty() && p[0] == kExecutableResetComplete;
  }
  return packet.channel == kControlChannel && p.size() >= 3 && p[0] == kCommandResponse &&
         p[2] == kInitializeUnsolicited;
}

}  // namespace bno08x

Bno08x::Bno08x(I2cTransport & transport)
: transport_(transport)
{
}

Bno08x::ReadResult Bno08x::readPacket(bno08x::Packet & packet)
{
  std::array<std::uint8_t, bno08x::kHeaderSize> header{};
  if (!transport_.read(header.data(), header.size())) {
    return ReadResult::kError;
  }
  const std::size_t length = static_cast<std::size_t>(header[0] | ((header[1] & 0x7F) << 8));
  if (length == 0) {
    return ReadResult::kEmpty;
  }
  if (length < bno08x::kHeaderSize || length > bno08x::kMaxPacketSize) {
    return ReadResult::kError;
  }
  packet.channel = header[2];
  packet.sequence = header[3];
  packet.payload.clear();
  if (length == bno08x::kHeaderSize) {
    return ReadResult::kPacket;
  }

  // The rest arrives behind a continuation header whose length covers that header plus the remaining cargo.
  std::vector<std::uint8_t> buffer(length);
  if (!transport_.read(buffer.data(), buffer.size())) {
    return ReadResult::kError;
  }
  const std::size_t continued = static_cast<std::size_t>(buffer[0] | ((buffer[1] & 0x7F) << 8));
  if (continued != length || buffer[2] != packet.channel) {
    return ReadResult::kError;
  }
  packet.payload.assign(buffer.begin() + bno08x::kHeaderSize, buffer.end());
  return ReadResult::kPacket;
}

bool Bno08x::sendPacket(std::uint8_t channel, const std::vector<std::uint8_t> & payload)
{
  if (channel >= sequence_.size() || payload.size() + bno08x::kHeaderSize > bno08x::kMaxPacketSize) {
    return false;
  }
  const std::size_t length = payload.size() + bno08x::kHeaderSize;
  std::vector<std::uint8_t> buffer{
    static_cast<std::uint8_t>(length & 0xFFU), static_cast<std::uint8_t>(length >> 8), channel,
    sequence_[channel]++};
  buffer.insert(buffer.end(), payload.begin(), payload.end());
  return transport_.write(buffer.data(), buffer.size());
}

template<typename Predicate>
bool Bno08x::waitForPacket(Predicate && matches, std::chrono::milliseconds timeout, bno08x::Packet & packet)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const ReadResult result = readPacket(packet);
    if (result == ReadResult::kPacket) {
      if (matches(packet)) {
        return true;
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  return false;
}

void Bno08x::drain(std::chrono::milliseconds limit)
{
  bno08x::Packet packet;
  const auto deadline = std::chrono::steady_clock::now() + limit;
  int empty_reads = 0;
  while (empty_reads < 3 && std::chrono::steady_clock::now() < deadline) {
    if (readPacket(packet) == ReadResult::kPacket) {
      empty_reads = 0;
    } else {
      ++empty_reads;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

bool Bno08x::softReset(std::chrono::milliseconds timeout)
{
  drain(kResetSettle);  // queued reports or an older "reset complete" must not be mistaken for this reset's reply
  if (!sendPacket(bno08x::kExecutableChannel, {bno08x::kExecutableReset})) {
    return false;
  }
  // Let the hub reboot before trusting a "reset complete".
  std::this_thread::sleep_for(kResetSettle);
  bno08x::Packet packet;
  const bool reset = waitForPacket(
    [](const bno08x::Packet & p) {
      return p.channel == bno08x::kExecutableChannel && !p.payload.empty() && p.payload[0] == 0x01;
    },
    timeout, packet);
  if (!reset) {
    return false;
  }
  drain(kResetSettle);  // the rest of this reset's messages (the Initialize response can arrive after it)
  return true;
}

std::optional<bno08x::ProductId> Bno08x::requestProductId(std::chrono::milliseconds timeout)
{
  if (!sendPacket(bno08x::kControlChannel, {bno08x::kProductIdRequest, 0x00})) {
    return std::nullopt;
  }
  bno08x::Packet packet;
  if (!waitForPacket([](const bno08x::Packet & p) {return bno08x::parseProductId(p).has_value();}, timeout, packet)) {
    return std::nullopt;
  }
  return bno08x::parseProductId(packet);
}

bool Bno08x::enableReport(std::uint8_t report_id, std::uint32_t interval_us)
{
  return sendPacket(bno08x::kControlChannel, bno08x::setFeatureCommand(report_id, interval_us));
}

}  // namespace autonom_hardware
