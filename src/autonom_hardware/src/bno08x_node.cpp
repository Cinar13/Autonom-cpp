#include "autonom_hardware/bno08x_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>

#include "autonom_hardware/parameter_utils.hpp"

namespace autonom_hardware
{
namespace
{

constexpr auto kWarnThrottleMs = 5000;
constexpr std::chrono::milliseconds kResetTimeout{1000};
constexpr std::chrono::milliseconds kRequestTimeout{300};
constexpr int kMaxPacketsPerPoll = 16;
// Reset notifications this soon after our own soft reset are its echoes, not a new reset of the sensor.
const rclcpp::Duration kResetGrace = rclcpp::Duration::from_seconds(1.5);

/// Declares a 3-element variance diagonal (same names as the bno055 driver) as a row-major 3x3 covariance.
std::array<double, 9> declareVariance3(
  rclcpp::Node & node, const std::string & name, const std::array<double, 3> & default_diagonal)
{
  const auto diagonal = declareParameter<std::vector<double>>(
    node, name, std::vector<double>(default_diagonal.begin(), default_diagonal.end()),
    "Variance diagonal (x, y, z) written into every message");
  if (diagonal.size() != 3) {
    throw std::invalid_argument(name + " must contain exactly 3 values");
  }
  std::array<double, 9> covariance{};
  for (std::size_t i = 0; i < 3; ++i) {
    // Zero would make the EKF trust the IMU completely and its rejection thresholds discard almost everything.
    if (!std::isfinite(diagonal[i]) || diagonal[i] <= 0.0) {
      throw std::invalid_argument(name + " values must be finite and positive");
    }
    covariance[i * 4] = diagonal[i];
  }
  return covariance;
}

}  // namespace

Bno08xNode::Bno08xNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("bno08x", options)
{
  declareParameters();
  const auto bus = declareParameter<std::string>(*this, "i2c_bus", "/dev/i2c-1", "I2C bus device");
  const auto address = declareParameter<std::int64_t>(*this, "i2c_address", 0x4B, "7-bit address (0x4A or 0x4B)");
  if (address < 0x08 || address > 0x77) {
    throw std::invalid_argument("i2c_address must be a 7-bit address");
  }
  transport_ = std::make_unique<I2cDevice>(bus, static_cast<std::uint8_t>(address));
  RCLCPP_INFO(get_logger(), "BNO08x on %s at 0x%02X", bus.c_str(), static_cast<unsigned int>(address));
  start();
}

Bno08xNode::Bno08xNode(const rclcpp::NodeOptions & options, std::unique_ptr<I2cTransport> transport)
: rclcpp::Node("bno08x", options),
  transport_(std::move(transport))
{
  declareParameters();
  start();
}

void Bno08xNode::declareParameters()
{
  frame_id_ = declareParameter<std::string>(*this, "frame_id", "imu_link", "frame_id of the published messages");
  const auto rate_hz = declareParameter<double>(*this, "report_rate_hz", 100.0, "Gyro/accel/orientation rate [Hz]");
  const auto poll_rate_hz = declareParameter<double>(*this, "poll_rate_hz", 400.0, "I2C polling rate [Hz]");
  const auto timeout_s = declareParameter<double>(
    *this, "report_timeout", 0.5, "Re-initialise the sensor when gyro reports stop for this long [s]");
  if (!(rate_hz > 0.0 && rate_hz <= 400.0) || !(poll_rate_hz >= rate_hz) || !(timeout_s > 0.0)) {
    throw std::invalid_argument("need 0 < report_rate_hz <= 400, poll_rate_hz >= report_rate_hz, report_timeout > 0");
  }
  report_interval_us_ = static_cast<std::uint32_t>(std::lround(1e6 / rate_hz));
  report_timeout_ = rclcpp::Duration::from_seconds(timeout_s);
  retry_period_ = rclcpp::Duration::from_seconds(1.0);

  orientation_covariance_ = declareVariance3(*this, "variance_orientation", {0.0159, 0.0159, 0.0159});
  angular_velocity_covariance_ = declareVariance3(*this, "variance_angular_vel", {0.0004, 0.0004, 0.0004});
  linear_acceleration_covariance_ = declareVariance3(*this, "variance_acc", {0.017, 0.017, 0.017});

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(20));
  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / poll_rate_hz), [this]() {poll();});
}

void Bno08xNode::start()
{
  sensor_ = std::make_unique<Bno08x>(*transport_);
  last_gyro_time_ = steady_clock_.now();
  last_init_attempt_ = steady_clock_.now();
  initialized_ = initialize();
}

bool Bno08xNode::initialize()
{
  have_acceleration_ = false;
  have_orientation_ = false;
  if (!sensor_->softReset(kResetTimeout)) {
    RCLCPP_ERROR(get_logger(), "BNO08x did not answer a soft reset; retrying every second");
    return false;
  }
  if (const auto id = sensor_->requestProductId(kRequestTimeout)) {
    RCLCPP_INFO(
      get_logger(), "BNO08x firmware %u.%u.%u, part %u build %u (reset cause %u)",
      static_cast<unsigned int>(id->version_major), static_cast<unsigned int>(id->version_minor),
      static_cast<unsigned int>(id->version_patch), id->part_number, id->build_number,
      static_cast<unsigned int>(id->reset_cause));
  } else {
    RCLCPP_WARN(get_logger(), "BNO08x sent no product ID; continuing");
  }
  for (const std::uint8_t report : {bno08x::kGyroscopeCalibrated, bno08x::kAccelerometer,
      bno08x::kGameRotationVector})
  {
    if (!sensor_->enableReport(report, report_interval_us_)) {
      RCLCPP_ERROR(get_logger(), "Failed to send Set Feature for report 0x%02X; retrying", static_cast<unsigned int>(report));
      return false;
    }
  }
  RCLCPP_INFO(get_logger(), "BNO08x reports enabled every %u us", report_interval_us_);
  last_gyro_time_ = steady_clock_.now();
  reset_grace_until_ = last_gyro_time_ + kResetGrace;
  return true;
}

void Bno08xNode::poll()
{
  const rclcpp::Time now = steady_clock_.now();
  if (!initialized_) {
    if (now - last_init_attempt_ >= retry_period_) {
      last_init_attempt_ = now;
      initialized_ = initialize();
    }
    return;
  }

  bno08x::Packet packet;
  std::vector<bno08x::SensorEvent> events;
  for (int i = 0; i < kMaxPacketsPerPoll; ++i) {
    const auto result = sensor_->readPacket(packet);
    if (result == Bno08x::ReadResult::kEmpty) {
      break;
    }
    if (result == Bno08x::ReadResult::kError) {
      ++read_errors_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, kWarnThrottleMs, "BNO08x I2C read failed (%lu in total)",
        static_cast<unsigned long>(read_errors_));
      break;
    }
    if (bno08x::isResetNotification(packet)) {
      if (now < reset_grace_until_) {
        continue;  // a late echo of our own soft reset, not a new reset
      }
      RCLCPP_WARN(get_logger(), "BNO08x reset itself; re-enabling reports");
      last_init_attempt_ = now;
      initialized_ = initialize();
      return;
    }
    if (const auto feature = bno08x::parseFeatureResponse(packet)) {
      RCLCPP_INFO(
        get_logger(), "BNO08x report 0x%02X granted every %u us", static_cast<unsigned int>(feature->report_id),
        feature->interval_us);
      continue;
    }
    if (packet.channel == bno08x::kInputReportChannel || packet.channel == bno08x::kWakeReportChannel) {
      events.clear();
      if (!bno08x::parseInputReports(packet.payload, events)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock_, kWarnThrottleMs, "BNO08x sent an unknown report; rest of packet skipped");
      }
      for (const auto & event : events) {
        handleEvent(event);
      }
    }
  }

  if (now - last_gyro_time_ > report_timeout_) {
    RCLCPP_WARN(get_logger(), "No BNO08x gyro report for %.2f s; re-initialising", (now - last_gyro_time_).seconds());
    last_init_attempt_ = now;
    initialized_ = initialize();
  }
}

void Bno08xNode::handleEvent(const bno08x::SensorEvent & event)
{
  switch (event.report_id) {
    case bno08x::kAccelerometer:
      latest_.linear_acceleration.x = event.vector[0];
      latest_.linear_acceleration.y = event.vector[1];
      latest_.linear_acceleration.z = event.vector[2];
      have_acceleration_ = true;
      break;
    case bno08x::kGameRotationVector: {
      const auto & q = event.quaternion;
      const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      if (norm > 0.5) {
        latest_.orientation.x = q[0] / norm;
        latest_.orientation.y = q[1] / norm;
        latest_.orientation.z = q[2] / norm;
        latest_.orientation.w = q[3] / norm;
        have_orientation_ = true;
      }
      break;
    }
    case bno08x::kGyroscopeCalibrated: {
      latest_.angular_velocity.x = event.vector[0];
      latest_.angular_velocity.y = event.vector[1];
      latest_.angular_velocity.z = event.vector[2];
      last_gyro_time_ = steady_clock_.now();
      if (!have_acceleration_ || !have_orientation_) {
        break;
      }
      auto msg = std::make_unique<sensor_msgs::msg::Imu>(latest_);
      msg->header.stamp = now();
      msg->header.frame_id = frame_id_;
      msg->orientation_covariance = orientation_covariance_;
      msg->angular_velocity_covariance = angular_velocity_covariance_;
      msg->linear_acceleration_covariance = linear_acceleration_covariance_;
      imu_pub_->publish(std::move(msg));
      break;
    }
    default:
      break;
  }
}

}  // namespace autonom_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(autonom_hardware::Bno08xNode)
