// BNO08x SHTP/SH-2 driver against a fake sensor that mimics the robot's chip (byte sequences from an
// i2ctransfer session on the robot, 2026-11-26): continuation headers, reset messages, product ID, feature replies.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "autonom_hardware/bno08x.hpp"
#include "autonom_hardware/bno08x_node.hpp"

using autonom_hardware::Bno08x;
namespace bno08x = autonom_hardware::bno08x;
using namespace std::chrono_literals;

namespace
{

using Bytes = std::vector<std::uint8_t>;

void putI16(Bytes & out, int value)
{
  const auto raw = static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
  out.push_back(static_cast<std::uint8_t>(raw & 0xFF));
  out.push_back(static_cast<std::uint8_t>(raw >> 8));
}

Bytes vectorReport(std::uint8_t id, std::uint8_t accuracy, int x, int y, int z)
{
  Bytes r{id, 0x00, accuracy, 0x00};
  putI16(r, x);
  putI16(r, y);
  putI16(r, z);
  return r;
}

Bytes gameRotationReport(int i, int j, int k, int real)
{
  Bytes r{bno08x::kGameRotationVector, 0x00, 0x03, 0x00};
  putI16(r, i);
  putI16(r, j);
  putI16(r, k);
  putI16(r, real);
  return r;
}

Bytes inputPacket(std::initializer_list<Bytes> reports)
{
  Bytes payload{0xFB, 0x10, 0x00, 0x00, 0x00};  // base timestamp
  for (const auto & r : reports) {
    payload.insert(payload.end(), r.begin(), r.end());
  }
  return payload;
}

/// Fake BNO08x: queued packets are read back the way the chip sends them; writes are recorded and answered.
class FakeBno08x : public autonom_hardware::I2cTransport
{
public:
  struct Written
  {
    std::uint8_t channel;
    std::uint8_t sequence;
    Bytes payload;
  };

  bool answer_reset = true;
  bool answer_features = true;
  bool fail_reads = false;
  std::vector<Written> written;

  void queue(std::uint8_t channel, Bytes payload) {outgoing_.push_back({channel, std::move(payload)});}

  bool read(std::uint8_t * data, std::size_t length) noexcept override
  {
    if (fail_reads) {
      return false;
    }
    std::fill(data, data + length, 0);
    if (!current_) {
      if (outgoing_.empty()) {
        return true;  // nothing queued: an all-zero header
      }
      current_ = outgoing_.front();
      outgoing_.pop_front();
      offset_ = 0;
      started_ = false;
    }
    // Every read starts with a header covering what is left; later reads set the continuation bit.
    const std::size_t remaining = current_->payload.size() - offset_;
    const std::size_t total = remaining + bno08x::kHeaderSize;
    data[0] = static_cast<std::uint8_t>(total & 0xFF);
    data[1] = static_cast<std::uint8_t>((total >> 8) | (started_ ? 0x80 : 0x00));
    started_ = true;
    data[2] = current_->channel;
    data[3] = sequence_[current_->channel];
    const std::size_t cargo = std::min(remaining, length - bno08x::kHeaderSize);
    std::copy_n(current_->payload.begin() + static_cast<std::ptrdiff_t>(offset_), cargo, data + 4);
    offset_ += cargo;
    if (offset_ == current_->payload.size() && length >= bno08x::kHeaderSize) {
      ++sequence_[current_->channel];
      current_.reset();
    }
    return true;
  }

  bool write(const std::uint8_t * data, std::size_t length) noexcept override
  {
    const std::size_t declared = static_cast<std::size_t>(data[0] | (data[1] << 8));
    if (length < 4 || declared != length) {
      return false;
    }
    Written w{data[2], data[3], Bytes(data + 4, data + length)};
    written.push_back(w);
    respond(w);
    return true;
  }

private:
  struct Pending
  {
    std::uint8_t channel;
    Bytes payload;
  };

  void respond(const Written & w)
  {
    if (w.channel == bno08x::kExecutableChannel && w.payload == Bytes{0x01} && answer_reset) {
      outgoing_.clear();
      queue(bno08x::kControlChannel, {0xF1, 0x00, 0x84, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00});
      queue(bno08x::kExecutableChannel, {0x01});
    } else if (w.channel == bno08x::kControlChannel && !w.payload.empty() && w.payload[0] == 0xF9) {
      queue(bno08x::kControlChannel, {0xF8, 0x02, 0x02, 0x06, 0xDC, 0xA4, 0x98, 0x00, 0xC1, 0x00, 0x00, 0x00,
          0x06, 0x00, 0x00, 0x00});
    } else if (w.channel == bno08x::kControlChannel && w.payload.size() == 17 && w.payload[0] == 0xFD &&
      answer_features)
    {
      Bytes reply(w.payload);
      reply[0] = 0xFC;
      queue(bno08x::kControlChannel, reply);
    }
  }

  std::deque<Pending> outgoing_;
  std::optional<Pending> current_;
  std::size_t offset_{0};
  bool started_{false};
  std::uint8_t sequence_[6]{};
};

TEST(Bno08xProtocol, SetFeatureCommandLayout)
{
  const auto command = bno08x::setFeatureCommand(bno08x::kGyroscopeCalibrated, 10000);
  ASSERT_EQ(command.size(), 17u);
  EXPECT_EQ(command[0], 0xFD);
  EXPECT_EQ(command[1], 0x02);
  EXPECT_EQ(command[5], 0x10);  // 10000 = 0x2710, little endian
  EXPECT_EQ(command[6], 0x27);
  EXPECT_EQ(command[7], 0x00);
  for (std::size_t i = 9; i < 17; ++i) {
    EXPECT_EQ(command[i], 0) << i;
  }
}

TEST(Bno08xProtocol, ParsesAndScalesReports)
{
  const auto payload = inputPacket({
    vectorReport(bno08x::kAccelerometer, 0x03, 256, -512, 2511),      // Q8
    vectorReport(bno08x::kGyroscopeCalibrated, 0x02, 0, 256, -1024),  // Q9
    gameRotationReport(0, 0, 11585, 11585),                           // 90 deg yaw, Q14
  });
  std::vector<bno08x::SensorEvent> events;
  ASSERT_TRUE(bno08x::parseInputReports(payload, events));
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].report_id, bno08x::kAccelerometer);
  EXPECT_EQ(events[0].accuracy, 3);
  EXPECT_DOUBLE_EQ(events[0].vector[0], 1.0);
  EXPECT_DOUBLE_EQ(events[0].vector[1], -2.0);
  EXPECT_NEAR(events[0].vector[2], 9.81, 0.01);
  EXPECT_EQ(events[1].accuracy, 2);
  EXPECT_DOUBLE_EQ(events[1].vector[1], 0.5);
  EXPECT_DOUBLE_EQ(events[1].vector[2], -2.0);
  EXPECT_NEAR(events[2].quaternion[2], std::sqrt(0.5), 1e-4);
  EXPECT_NEAR(events[2].quaternion[3], std::sqrt(0.5), 1e-4);
}

TEST(Bno08xProtocol, StopsAtUnknownOrTruncatedReport)
{
  auto payload = inputPacket({vectorReport(bno08x::kAccelerometer, 0, 1, 2, 3)});
  payload.push_back(0x7E);  // unknown ID
  std::vector<bno08x::SensorEvent> events;
  EXPECT_FALSE(bno08x::parseInputReports(payload, events));
  EXPECT_EQ(events.size(), 1u);

  auto truncated = inputPacket({vectorReport(bno08x::kGyroscopeCalibrated, 0, 1, 2, 3)});
  truncated.resize(truncated.size() - 2);
  events.clear();
  EXPECT_FALSE(bno08x::parseInputReports(truncated, events));
  EXPECT_TRUE(events.empty());
}

TEST(Bno08xProtocol, RecognisesResetMessages)
{
  EXPECT_TRUE(bno08x::isResetNotification({bno08x::kExecutableChannel, 0, {0x01}}));
  EXPECT_TRUE(bno08x::isResetNotification({bno08x::kControlChannel, 0, {0xF1, 0x00, 0x84}}));
  EXPECT_FALSE(bno08x::isResetNotification({bno08x::kControlChannel, 0, {0xF1, 0x00, 0x04}}));
  EXPECT_FALSE(bno08x::isResetNotification({bno08x::kInputReportChannel, 0, {0x01}}));
}

TEST(Bno08xDriver, ReadsPacketsBehindContinuationHeaders)
{
  FakeBno08x fake;
  Bno08x sensor(fake);
  bno08x::Packet packet;
  EXPECT_EQ(sensor.readPacket(packet), Bno08x::ReadResult::kEmpty);

  Bytes big(300);
  for (std::size_t i = 0; i < big.size(); ++i) {
    big[i] = static_cast<std::uint8_t>(i);
  }
  fake.queue(bno08x::kInputReportChannel, big);
  fake.queue(bno08x::kExecutableChannel, {0x01});
  ASSERT_EQ(sensor.readPacket(packet), Bno08x::ReadResult::kPacket);
  EXPECT_EQ(packet.channel, bno08x::kInputReportChannel);
  EXPECT_EQ(packet.payload, big);
  ASSERT_EQ(sensor.readPacket(packet), Bno08x::ReadResult::kPacket);
  EXPECT_EQ(packet.channel, bno08x::kExecutableChannel);
  EXPECT_EQ(packet.payload, Bytes{0x01});
  EXPECT_EQ(sensor.readPacket(packet), Bno08x::ReadResult::kEmpty);

  fake.fail_reads = true;
  EXPECT_EQ(sensor.readPacket(packet), Bno08x::ReadResult::kError);
}

TEST(Bno08xDriver, ResetProductIdAndEnableReport)
{
  FakeBno08x fake;
  Bno08x sensor(fake);
  ASSERT_TRUE(sensor.softReset(500ms));
  const auto id = sensor.requestProductId(500ms);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->version_major, 2);
  EXPECT_EQ(id->version_minor, 6);
  EXPECT_EQ(id->part_number, 10003676u);
  EXPECT_EQ(id->build_number, 193u);
  EXPECT_EQ(id->version_patch, 6);

  ASSERT_TRUE(sensor.enableReport(bno08x::kGyroscopeCalibrated, 10000));
  EXPECT_EQ(fake.written.back().payload, bno08x::setFeatureCommand(bno08x::kGyroscopeCalibrated, 10000));

  // Sequence numbers count per channel.
  ASSERT_EQ(fake.written.size(), 3u);
  EXPECT_EQ(fake.written[0].channel, bno08x::kExecutableChannel);
  EXPECT_EQ(fake.written[0].sequence, 0);
  EXPECT_EQ(fake.written[1].channel, bno08x::kControlChannel);
  EXPECT_EQ(fake.written[1].sequence, 0);
  EXPECT_EQ(fake.written[2].sequence, 1);
}

TEST(Bno08xDriver, ResetIgnoresStaleResetComplete)
{
  FakeBno08x fake;
  fake.answer_reset = false;  // the sensor is still rebooting: only an old message is queued
  fake.queue(bno08x::kExecutableChannel, {0x01});
  Bno08x sensor(fake);
  // The stale message is drained before the reset is sent, so it can't pass as this reset's reply.
  EXPECT_FALSE(sensor.softReset(100ms));
}

TEST(Bno08xDriver, TimesOutWithoutAnswers)
{
  FakeBno08x fake;
  fake.answer_reset = false;
  Bno08x sensor(fake);
  EXPECT_FALSE(sensor.softReset(50ms));
}

class Bno08xNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(Bno08xNodeTest, PublishesImuWithCovariancesAndReenablesAfterReset)
{
  auto fake_owner = std::make_unique<FakeBno08x>();
  FakeBno08x * fake = fake_owner.get();
  // A long report timeout: the fake streams nothing between packets, and this test is about the reset path.
  auto node = std::make_shared<autonom_hardware::Bno08xNode>(
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("report_timeout", 10.0)}), std::move(fake_owner));
  // Initialisation: reset, product ID, three Set Feature commands.
  ASSERT_EQ(fake->written.size(), 5u);

  auto io = std::make_shared<rclcpp::Node>("bno08x_test_io");
  std::vector<sensor_msgs::msg::Imu> received;
  auto sub = io->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::QoS(20), [&received](const sensor_msgs::msg::Imu & msg) {received.push_back(msg);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io);

  auto spin_until = [&executor](const std::function<bool()> & done, std::chrono::milliseconds limit = 3000ms) {
      const auto deadline = std::chrono::steady_clock::now() + limit;
      while (!done() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(10ms);
      }
    };

  // A gyro report before any accelerometer/orientation publishes nothing.
  fake->queue(bno08x::kInputReportChannel, inputPacket({vectorReport(bno08x::kGyroscopeCalibrated, 3, 0, 0, 512)}));
  spin_until([&]() {return false;}, 300ms);  // let it poll for a while
  EXPECT_TRUE(received.empty());

  fake->queue(bno08x::kInputReportChannel, inputPacket({
      vectorReport(bno08x::kAccelerometer, 3, 0, 0, 2511),
      gameRotationReport(0, 0, 0, 16384),
      vectorReport(bno08x::kGyroscopeCalibrated, 3, 0, 0, 256),
    }));
  spin_until([&]() {return !received.empty();});
  ASSERT_FALSE(received.empty());
  const auto & msg = received.back();
  EXPECT_EQ(msg.header.frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(msg.angular_velocity.z, 0.5);
  EXPECT_NEAR(msg.linear_acceleration.z, 9.81, 0.01);
  EXPECT_DOUBLE_EQ(msg.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[8], 0.0004);
  EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[0], 0.017);
  EXPECT_DOUBLE_EQ(msg.orientation_covariance[4], 0.0159);
  EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[1], 0.0);

  // Within the grace period after our own reset, a reset message is treated as its echo and ignored.
  const std::size_t before = fake->written.size();
  fake->queue(bno08x::kExecutableChannel, {0x01});
  spin_until([&]() {return false;}, 300ms);
  EXPECT_EQ(fake->written.size(), before);
  // After it, the node must reset the sensor cleanly and send the three Set Feature commands again.
  spin_until([&]() {return false;}, 1300ms);
  fake->queue(bno08x::kExecutableChannel, {0x01});
  spin_until([&]() {return fake->written.size() >= before + 5;});
  EXPECT_EQ(fake->written.size(), before + 5);
}

TEST_F(Bno08xNodeTest, RejectsZeroVariance)
{
  EXPECT_THROW(
    autonom_hardware::Bno08xNode(
      rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("variance_acc", std::vector<double>{0.0, 0.017, 0.017})}),
      std::make_unique<FakeBno08x>()),
    std::invalid_argument);
}

}  // namespace
