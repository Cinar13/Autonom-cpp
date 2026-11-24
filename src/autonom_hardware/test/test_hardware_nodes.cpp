// Runs the real nodes with the real config/hardware.yaml against the fake GPIO chip and a fake sysfs PWM chip.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>

#include "autonom_hardware/motor_driver_node.hpp"
#include "autonom_hardware/start_button_node.hpp"
#include "autonom_hardware/wheel_odometry_node.hpp"
#include "fake_gpiod.hpp"
#include "fake_pwm_chip.hpp"

using fake_gpiod::Mode;
using namespace std::chrono_literals;

namespace
{

// The wiring schematic (BCM numbers).
constexpr unsigned int kAin1 = 5, kAin2 = 6, kBin1 = 20, kBin2 = 21, kStby = 26;
constexpr unsigned int kLeftEncA = 17, kLeftEncB = 27, kRightEncA = 23, kRightEncB = 24;
constexpr unsigned int kStartButton = 16;
constexpr unsigned int kPwmLeft = 0, kPwmRight = 1;  // GPIO12 / GPIO13 via dtoverlay=pwm-2chan

int level(unsigned int offset) { return fake_gpiod::line(offset).value; }

}  // namespace

class HardwareNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fake_gpiod::reset();
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    executor_->add_node(helper_);
  }

  void TearDown() override
  {
    helper_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  rclcpp::NodeOptions options() const
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.arguments({"--ros-args", "--params-file", HARDWARE_YAML});
    // NodeOptions overrides beat the node's section of the params file; only the PWM sysfs path changes.
    options.parameter_overrides({
      {"left_motor.pwm_chip", pwm_.path()},
      {"right_motor.pwm_chip", pwm_.path()}});
    return options;
  }

  template<typename NodeT>
  std::shared_ptr<NodeT> make()
  {
    auto node = std::make_shared<NodeT>(options());
    executor_->add_node(node);
    return node;
  }

  /// Spins for `duration`, calling `each` (if given) every millisecond.
  void spinFor(std::chrono::milliseconds duration, const std::function<void()> & each = {})
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (each) {
        each();
      }
      executor_->spin_some(1ms);
      std::this_thread::sleep_for(1ms);
    }
  }

  bool spinUntil(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 2000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!condition()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      executor_->spin_some(1ms);
      std::this_thread::sleep_for(1ms);
    }
    return true;
  }

  fake_pwm::FakePwmChip pwm_;
  rclcpp::Node::SharedPtr helper_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(HardwareNodesTest, AllNodesShareOneProcessWithTheSchematicPinout)
{
  auto odometry = make<autonom_hardware::WheelOdometryNode>();
  auto motors = make<autonom_hardware::MotorDriverNode>();
  auto button = make<autonom_hardware::StartButtonNode>();

  for (unsigned int pin : {kAin1, kAin2, kBin1, kBin2, kStby}) {
    EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kOutput) << "GPIO" << pin;
    EXPECT_EQ(level(pin), 0) << "GPIO" << pin << " must start low (motors off, TB6612 in standby)";
  }
  EXPECT_EQ(fake_gpiod::line(kAin1).consumer, "autonom_left_motor_in1");
  EXPECT_EQ(fake_gpiod::line(kAin2).consumer, "autonom_left_motor_in2");
  EXPECT_EQ(fake_gpiod::line(kBin1).consumer, "autonom_right_motor_in1");
  EXPECT_EQ(fake_gpiod::line(kBin2).consumer, "autonom_right_motor_in2");
  EXPECT_EQ(fake_gpiod::line(kStby).consumer, "autonom_motor_standby");
  for (unsigned int pin : {kLeftEncA, kLeftEncB, kRightEncA, kRightEncB}) {
    EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kEdgeEvents) << "GPIO" << pin;
    EXPECT_TRUE(fake_gpiod::line(pin).pull_up) << "GPIO" << pin;
  }
  EXPECT_EQ(fake_gpiod::line(kStartButton).mode, Mode::kInput);
  EXPECT_TRUE(fake_gpiod::line(kStartButton).pull_up);
  EXPECT_EQ(fake_gpiod::line(kStartButton).consumer, "autonom_start_button");

  int requested = 0;
  for (unsigned int pin = 0; pin < fake_gpiod::kNumLines; ++pin) {
    requested += fake_gpiod::line(pin).mode != Mode::kFree ? 1 : 0;
  }
  EXPECT_EQ(requested, 10) << "only the schematic's pins may be claimed";
  for (unsigned int pin : {12u, 13u}) {
    EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kFree) << "GPIO" << pin << " belongs to the PWM overlay";
  }

  for (unsigned int channel : {kPwmLeft, kPwmRight}) {
    EXPECT_EQ(pwm_.readNumber(channel, "enable"), 1);
    EXPECT_EQ(pwm_.readNumber(channel, "period"), 50000);
    EXPECT_EQ(pwm_.readNumber(channel, "duty_cycle"), 0);
  }
}

TEST_F(HardwareNodesTest, CodeDefaultsMatchTheSchematicWithoutTheParamsFile)
{
  // A node started without hardware.yaml (e.g. `ros2 run`) must still use the schematic's pins.
  rclcpp::NodeOptions bare;
  bare.use_global_arguments(false);
  bare.parameter_overrides({{"left_motor.pwm_chip", pwm_.path()}, {"right_motor.pwm_chip", pwm_.path()}});
  autonom_hardware::WheelOdometryNode odometry(bare);
  autonom_hardware::MotorDriverNode motors(bare);
  autonom_hardware::StartButtonNode button(bare);

  const std::map<unsigned int, std::string> expected{
    {kAin1, "autonom_left_motor_in1"}, {kAin2, "autonom_left_motor_in2"},
    {kBin1, "autonom_right_motor_in1"}, {kBin2, "autonom_right_motor_in2"},
    {kStby, "autonom_motor_standby"},
    {kLeftEncA, "autonom_left_encoder"}, {kLeftEncB, "autonom_left_encoder"},
    {kRightEncA, "autonom_right_encoder"}, {kRightEncB, "autonom_right_encoder"},
    {kStartButton, "autonom_start_button"}};
  for (unsigned int pin = 0; pin < fake_gpiod::kNumLines; ++pin) {
    const auto it = expected.find(pin);
    if (it == expected.end()) {
      EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kFree) << "GPIO" << pin << " claimed by default";
    } else {
      EXPECT_EQ(fake_gpiod::line(pin).consumer, it->second) << "GPIO" << pin;
    }
  }
  EXPECT_EQ(pwm_.readNumber(kPwmLeft, "enable"), 1);
  EXPECT_EQ(pwm_.readNumber(kPwmRight, "enable"), 1);
}

class MotorDriverTest : public HardwareNodesTest
{
protected:
  void SetUp() override
  {
    HardwareNodesTest::SetUp();
    motors_ = make<autonom_hardware::MotorDriverNode>();
    cmd_pub_ = helper_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    joint_pub_ = helper_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  void TearDown() override
  {
    motors_.reset();
    HardwareNodesTest::TearDown();
  }

  void publish(double linear, double angular, bool feedback = true)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
    if (feedback) {
      sensor_msgs::msg::JointState joints;
      joints.name = {"left_wheel_joint", "right_wheel_joint"};
      joints.position = {0.0, 0.0};
      joints.velocity = {0.0, 0.0};
      joint_pub_->publish(joints);
    }
  }

  void drive(double linear, double angular, std::chrono::milliseconds duration = 200ms)
  {
    int tick = 0;
    spinFor(duration, [&] {
      if (tick++ % 10 == 0) {
        publish(linear, angular);
      }
    });
  }

  long long duty(unsigned int channel) const { return pwm_.readNumber(channel, "duty_cycle"); }

  std::shared_ptr<autonom_hardware::MotorDriverNode> motors_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

TEST_F(MotorDriverTest, ForwardEnablesTheBridgeAndDrivesBothWheelsForward)
{
  drive(0.2, 0.0);
  EXPECT_EQ(level(kStby), 1);
  EXPECT_EQ(level(kAin1), 1);
  EXPECT_EQ(level(kAin2), 0);
  // right_motor.inverted: true (mirror-mounted), so its forward is BIN2.
  EXPECT_EQ(level(kBin1), 0);
  EXPECT_EQ(level(kBin2), 1);
  EXPECT_GT(duty(kPwmLeft), 0);
  EXPECT_GT(duty(kPwmRight), 0);
}

TEST_F(MotorDriverTest, TurningInPlaceDrivesTheWheelsOppositeWays)
{
  drive(0.0, 2.0);
  EXPECT_EQ(level(kStby), 1);
  EXPECT_EQ(level(kAin1), 0);  // left backwards
  EXPECT_EQ(level(kAin2), 1);
  EXPECT_EQ(level(kBin1), 0);  // right forwards (inverted -> BIN2)
  EXPECT_EQ(level(kBin2), 1);
}

TEST_F(MotorDriverTest, CmdVelTimeoutStopsEverythingAndEntersStandby)
{
  drive(0.2, 0.0);
  ASSERT_EQ(level(kStby), 1);
  spinFor(700ms);  // cmd_vel_timeout is 0.5 s
  EXPECT_EQ(level(kStby), 0);
  for (unsigned int pin : {kAin1, kAin2, kBin1, kBin2}) {
    EXPECT_EQ(level(pin), 0) << "GPIO" << pin;
  }
  EXPECT_EQ(duty(kPwmLeft), 0);
  EXPECT_EQ(duty(kPwmRight), 0);
}

TEST_F(MotorDriverTest, NoEncoderFeedbackKeepsTheMotorsOff)
{
  int tick = 0;
  spinFor(300ms, [&] {
    if (tick++ % 10 == 0) {
      publish(0.2, 0.0, false);
    }
  });
  EXPECT_EQ(level(kStby), 0);
  EXPECT_EQ(duty(kPwmLeft), 0);
  EXPECT_EQ(duty(kPwmRight), 0);
}

TEST_F(MotorDriverTest, ZeroCommandStopsImmediately)
{
  drive(0.2, 0.0);
  ASSERT_EQ(level(kStby), 1);
  drive(0.0, 0.0, 100ms);
  EXPECT_EQ(level(kStby), 0);
  EXPECT_EQ(duty(kPwmLeft), 0);
}

TEST_F(MotorDriverTest, NonFiniteCommandIsIgnored)
{
  drive(std::nan(""), 0.0, 100ms);
  EXPECT_EQ(level(kStby), 0);
  EXPECT_EQ(duty(kPwmLeft), 0);
}

TEST_F(MotorDriverTest, ShutdownReleasesEverythingLowAndDisablesPwm)
{
  drive(0.2, 0.0);
  ASSERT_EQ(level(kStby), 1);
  executor_->remove_node(motors_);
  motors_.reset();
  for (unsigned int pin : {kAin1, kAin2, kBin1, kBin2, kStby}) {
    EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kFree) << "GPIO" << pin;
    EXPECT_EQ(level(pin), 0) << "GPIO" << pin;
  }
  // Both channels end disabled at duty 0. (Unexport itself is covered by test_pwm_channel: the fake chip's
  // polling watcher cannot tell apart two unexport writes made microseconds apart.)
  for (unsigned int channel : {kPwmLeft, kPwmRight}) {
    if (pwm_.exists(channel)) {
      EXPECT_EQ(pwm_.readNumber(channel, "enable"), 0) << "pwm" << channel;
      EXPECT_EQ(pwm_.readNumber(channel, "duty_cycle"), 0) << "pwm" << channel;
    }
  }
}

class StartButtonTest : public HardwareNodesTest
{
protected:
  void subscribe()
  {
    sub_ = helper_->create_subscription<std_msgs::msg::Empty>(
      "race/start_signal", rclcpp::QoS(10).reliable(), [this](const std_msgs::msg::Empty &) { ++presses_; });
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  int presses_{0};
};

TEST_F(StartButtonTest, EachPressPublishesOnce)
{
  subscribe();
  auto button = make<autonom_hardware::StartButtonNode>();
  spinFor(100ms);  // released (pull-up reads high)
  EXPECT_EQ(presses_, 0);

  fake_gpiod::setInputLevel(kStartButton, 0);
  ASSERT_TRUE(spinUntil([&] { return presses_ == 1; }));
  spinFor(200ms);  // holding it down does not repeat
  EXPECT_EQ(presses_, 1);

  fake_gpiod::setInputLevel(kStartButton, 1);
  spinFor(100ms);
  fake_gpiod::setInputLevel(kStartButton, 0);
  ASSERT_TRUE(spinUntil([&] { return presses_ == 2; }));
}

TEST_F(StartButtonTest, ShortGlitchIsNotAPress)
{
  subscribe();
  auto button = make<autonom_hardware::StartButtonNode>();
  spinFor(100ms);
  fake_gpiod::setInputLevel(kStartButton, 0);
  spinFor(8ms);  // well under debounce_ms (30)
  fake_gpiod::setInputLevel(kStartButton, 1);
  spinFor(150ms);
  EXPECT_EQ(presses_, 0);
}

TEST_F(StartButtonTest, ButtonHeldAtStartupDoesNotStartTheRace)
{
  subscribe();
  fake_gpiod::setInputLevel(kStartButton, 0);
  auto button = make<autonom_hardware::StartButtonNode>();
  spinFor(300ms);
  EXPECT_EQ(presses_, 0);

  fake_gpiod::setInputLevel(kStartButton, 1);
  spinFor(100ms);
  fake_gpiod::setInputLevel(kStartButton, 0);
  ASSERT_TRUE(spinUntil([&] { return presses_ == 1; }));
}

TEST_F(HardwareNodesTest, WheelOdometryTurnsEncoderEdgesIntoWheelAngle)
{
  for (unsigned int pin : {kLeftEncA, kLeftEncB, kRightEncA, kRightEncB}) {
    fake_gpiod::setInputLevel(pin, 0);
  }
  std::optional<sensor_msgs::msg::JointState> joints;
  std::optional<nav_msgs::msg::Odometry> odom;
  auto joint_sub = helper_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](const sensor_msgs::msg::JointState & msg) { joints = msg; });
  auto odom_sub = helper_->create_subscription<nav_msgs::msg::Odometry>(
    "wheel/odom", 10, [&](const nav_msgs::msg::Odometry & msg) { odom = msg; });
  auto odometry = make<autonom_hardware::WheelOdometryNode>();

  // One wheel revolution forward on both sides: 1320 x4 ticks = 330 cycles. The decoder counts B leading A
  // as positive; the right encoder is mirrored (right_encoder.inverted: true), so its forward is A leading B.
  std::vector<fake_gpiod::Edge> edges;
  std::int64_t t = 1'000'000'000;
  for (int i = 0; i < 330; ++i) {
    for (const auto & [pin, rising] : std::vector<std::pair<unsigned int, bool>>{
        {kLeftEncB, true}, {kLeftEncA, true}, {kLeftEncB, false}, {kLeftEncA, false},
        {kRightEncA, true}, {kRightEncB, true}, {kRightEncA, false}, {kRightEncB, false}})
    {
      edges.push_back({pin, rising, t += 1000});
    }
  }
  fake_gpiod::pushEdges(edges);

  ASSERT_TRUE(spinUntil([&] {
    return joints && joints->position.size() == 2 && std::abs(joints->position[0] - 2.0 * M_PI) < 1e-9 &&
      std::abs(joints->position[1] - 2.0 * M_PI) < 1e-9;
  })) << (joints ? std::to_string(joints->position[0]) + ", " + std::to_string(joints->position[1]) : "no joint_states");
  EXPECT_EQ(joints->name, (std::vector<std::string>{"left_wheel_joint", "right_wheel_joint"}));
  ASSERT_TRUE(spinUntil([&] { return odom && odom->pose.pose.position.x > 0.0; }));
  EXPECT_NEAR(odom->pose.pose.position.x, 2.0 * M_PI * 0.0325, 1e-6);  // one circumference straight ahead
  EXPECT_NEAR(odom->pose.pose.position.y, 0.0, 1e-9);
  EXPECT_EQ(odom->header.frame_id, "odom");
  EXPECT_EQ(odom->child_frame_id, "base_footprint");
}

TEST_F(HardwareNodesTest, MissingGpioChipFailsLoudly)
{
  rclcpp::NodeOptions bad = options();
  auto overrides = bad.parameter_overrides();
  overrides.emplace_back("gpio_chip", "no-such-chip");
  bad.parameter_overrides(overrides);
  EXPECT_ANY_THROW(autonom_hardware::MotorDriverNode{bad});
  EXPECT_ANY_THROW(autonom_hardware::WheelOdometryNode{bad});
  EXPECT_ANY_THROW(autonom_hardware::StartButtonNode{bad});
  for (unsigned int pin = 0; pin < fake_gpiod::kNumLines; ++pin) {
    EXPECT_EQ(fake_gpiod::line(pin).mode, Mode::kFree) << "GPIO" << pin << " leaked";
  }
}
