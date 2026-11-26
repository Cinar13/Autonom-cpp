// Pinout checks: the wiring schematic is written down once below, and everything else must agree with it:
// the Raspberry Pi 40-pin header, config/hardware.yaml, autonom_bringup/config/imu.yaml, the README pinout
// table and the pin comments in hardware.yaml. test_hardware_nodes then checks the nodes claim these pins.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{

// Raspberry Pi 40-pin header (identical on Pi 5): BCM GPIO -> physical pin.
const std::map<int, int> kHeaderPinOfGpio{
  {2, 3}, {3, 5}, {4, 7}, {14, 8}, {15, 10}, {17, 11}, {18, 12}, {27, 13}, {22, 15}, {23, 16}, {24, 18},
  {10, 19}, {9, 21}, {25, 22}, {11, 23}, {8, 24}, {7, 26}, {0, 27}, {1, 28}, {5, 29}, {6, 31}, {12, 32},
  {13, 33}, {19, 35}, {16, 36}, {26, 37}, {20, 38}, {21, 40}};
const std::set<int> kHeader3v3{1, 17};
const std::set<int> kHeaderGnd{6, 9, 14, 20, 25, 30, 34, 39};
// GPIO0/1 (pins 27/28) are the HAT ID EEPROM bus; leave them alone.
const std::set<int> kReservedGpio{0, 1};
// RP1 PWM0 channel available on a GPIO (alternate function a0 / a3).
const std::map<int, int> kPwm0ChannelOfGpio{{12, 0}, {13, 1}, {18, 2}, {19, 3}};

struct Connection
{
  std::string function;
  int gpio;
  int header_pin;
};

// The wiring schematic (Raspberry Pi 5 side).
const std::vector<Connection> kSchematic{
  {"TB6612 PWMA (left motor)", 12, 32},
  {"TB6612 AIN1 (left motor)", 5, 29},
  {"TB6612 AIN2 (left motor)", 6, 31},
  {"TB6612 STBY", 26, 37},
  {"TB6612 BIN1 (right motor)", 20, 38},
  {"TB6612 BIN2 (right motor)", 21, 40},
  {"TB6612 PWMB (right motor)", 13, 33},
  {"Left encoder A", 23, 16},
  {"Left encoder B", 24, 18},
  {"Right encoder A", 17, 11},
  {"Right encoder B", 27, 13},
  {"BNO055 SDA", 2, 3},
  {"BNO055 SCL", 3, 5},
  {"Start button", 16, 36},
};
// Power taps on the header: TB6612 VCC / GND, and the 3.3 V / GND logic rails (encoders, BNO055).
constexpr int kTb6612Vcc = 1, kTb6612Gnd = 6, kLogic3v3 = 17, kLogicGnd = 14;

int gpioOf(const std::string & function)
{
  for (const auto & c : kSchematic) {
    if (c.function == function) {
      return c.gpio;
    }
  }
  throw std::out_of_range(function);
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

YAML::Node hardwareYaml() { return YAML::LoadFile(HARDWARE_YAML); }

}  // namespace

TEST(Pinout, SchematicMatchesTheRaspberryPiHeader)
{
  for (const auto & c : kSchematic) {
    ASSERT_TRUE(kHeaderPinOfGpio.count(c.gpio)) << c.function << ": GPIO" << c.gpio << " is not on the header";
    EXPECT_EQ(kHeaderPinOfGpio.at(c.gpio), c.header_pin) << c.function << ": GPIO" << c.gpio;
    EXPECT_FALSE(kReservedGpio.count(c.gpio)) << c.function << " uses the ID EEPROM bus";
  }
  EXPECT_TRUE(kHeader3v3.count(kTb6612Vcc));
  EXPECT_TRUE(kHeader3v3.count(kLogic3v3));
  EXPECT_TRUE(kHeaderGnd.count(kTb6612Gnd));
  EXPECT_TRUE(kHeaderGnd.count(kLogicGnd));
}

TEST(Pinout, EveryGpioIsUsedOnce)
{
  std::map<int, std::string> owner;
  for (const auto & c : kSchematic) {
    const auto [it, inserted] = owner.emplace(c.gpio, c.function);
    EXPECT_TRUE(inserted) << "GPIO" << c.gpio << " used by both " << it->second << " and " << c.function;
  }
  for (int power_pin : {kTb6612Vcc, kTb6612Gnd, kLogic3v3, kLogicGnd}) {
    for (const auto & c : kSchematic) {
      EXPECT_NE(c.header_pin, power_pin) << c.function;
    }
  }
}

TEST(Pinout, MotorPwmPinsAreHardwarePwmChannels)
{
  const auto motor = hardwareYaml()["motor_driver"]["ros__parameters"];
  const int left_gpio = gpioOf("TB6612 PWMA (left motor)");
  const int right_gpio = gpioOf("TB6612 PWMB (right motor)");
  ASSERT_TRUE(kPwm0ChannelOfGpio.count(left_gpio));
  ASSERT_TRUE(kPwm0ChannelOfGpio.count(right_gpio));
  EXPECT_EQ(motor["left_motor"]["pwm_channel"].as<int>(), kPwm0ChannelOfGpio.at(left_gpio));
  EXPECT_EQ(motor["right_motor"]["pwm_channel"].as<int>(), kPwm0ChannelOfGpio.at(right_gpio));
  EXPECT_EQ(motor["left_motor"]["pwm_chip"].as<std::string>(), motor["right_motor"]["pwm_chip"].as<std::string>())
    << "both channels are on RP1 PWM0";

  // The config.txt overlay documented in hardware.yaml must route exactly these two GPIOs to PWM0.
  const std::string yaml = readFile(HARDWARE_YAML);
  std::smatch overlay;
  ASSERT_TRUE(std::regex_search(yaml, overlay, std::regex(R"(dtoverlay=pwm-2chan,pin=(\d+),func=4,pin2=(\d+),func2=4)")));
  EXPECT_EQ(std::stoi(overlay[1]), left_gpio);
  EXPECT_EQ(std::stoi(overlay[2]), right_gpio);
}

TEST(Pinout, HardwareYamlMatchesTheSchematic)
{
  const auto yaml = hardwareYaml();
  EXPECT_EQ(yaml["/**"]["ros__parameters"]["gpio_chip"].as<std::string>(), "pinctrl-rp1");

  const auto motor = yaml["motor_driver"]["ros__parameters"];
  EXPECT_EQ(motor["standby_pin"].as<int>(), gpioOf("TB6612 STBY"));
  EXPECT_EQ(motor["left_motor"]["in1_pin"].as<int>(), gpioOf("TB6612 AIN1 (left motor)"));
  EXPECT_EQ(motor["left_motor"]["in2_pin"].as<int>(), gpioOf("TB6612 AIN2 (left motor)"));
  EXPECT_EQ(motor["right_motor"]["in1_pin"].as<int>(), gpioOf("TB6612 BIN1 (right motor)"));
  EXPECT_EQ(motor["right_motor"]["in2_pin"].as<int>(), gpioOf("TB6612 BIN2 (right motor)"));

  const auto odometry = yaml["wheel_odometry"]["ros__parameters"];
  EXPECT_EQ(odometry["left_encoder"]["pin_a"].as<int>(), gpioOf("Left encoder A"));
  EXPECT_EQ(odometry["left_encoder"]["pin_b"].as<int>(), gpioOf("Left encoder B"));
  EXPECT_EQ(odometry["right_encoder"]["pin_a"].as<int>(), gpioOf("Right encoder A"));
  EXPECT_EQ(odometry["right_encoder"]["pin_b"].as<int>(), gpioOf("Right encoder B"));
  // Encoders run from 3.3 V without external pull-ups: the internal ones must be on.
  EXPECT_TRUE(odometry["left_encoder"]["pull_up"].as<bool>());
  EXPECT_TRUE(odometry["right_encoder"]["pull_up"].as<bool>());

  const auto button = yaml["start_button"]["ros__parameters"];
  EXPECT_EQ(button["pin"].as<int>(), gpioOf("Start button"));
  EXPECT_TRUE(button["pull_up"].as<bool>()) << "button to GND needs the pull-up";
  EXPECT_TRUE(button["active_low"].as<bool>()) << "button to GND reads low when pressed";
}

TEST(Pinout, ImuIsOnI2c1AtTheBno055Address)
{
  const std::string path = std::string(SOURCE_DIR) + "/../autonom_bringup/config/imu.yaml";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << path << " not found";
  }
  const auto imu = YAML::LoadFile(path)["/**"]["ros__parameters"];
  EXPECT_EQ(imu["connection_type"].as<std::string>(), "i2c");
  EXPECT_EQ(imu["i2c_bus"].as<int>(), 1) << "GPIO2/3 (pins 3/5) are I2C1";
  EXPECT_EQ(imu["i2c_addr"].as<int>(), 0x28);
  EXPECT_EQ(gpioOf("BNO055 SDA"), 2);
  EXPECT_EQ(gpioOf("BNO055 SCL"), 3);
}

TEST(Pinout, HardwareYamlCommentsMatchTheHeader)
{
  // Comment lines like "PWMA GPIO12 / 32" pair a BCM number with its header pin.
  const std::string yaml = readFile(HARDWARE_YAML);
  const std::regex pair(R"(GPIO(\d+) / (\d+))");
  int checked = 0;
  for (auto it = std::sregex_iterator(yaml.begin(), yaml.end(), pair); it != std::sregex_iterator(); ++it) {
    const int gpio = std::stoi((*it)[1]);
    const int header = std::stoi((*it)[2]);
    ASSERT_TRUE(kHeaderPinOfGpio.count(gpio)) << it->str();
    EXPECT_EQ(kHeaderPinOfGpio.at(gpio), header) << "hardware.yaml comment: " << it->str();
    ++checked;
  }
  EXPECT_GE(checked, 14);
}

TEST(Pinout, ReadmeTableMatchesTheSchematic)
{
  const std::string path = std::string(SOURCE_DIR) + "/../../README.md";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << path << " not found";
  }
  // Rows: | Function | 12 | 32 | ... | or | ... | 5 / 6 | 29 / 31 | ... |
  const std::regex row(R"(^\|([^|]+)\|\s*([\d /]+?)\s*\|\s*([\d /]+?)\s*\|)");
  const std::regex number(R"(\d+)");
  std::set<int> readme_gpios;
  std::istringstream lines(readFile(path));
  for (std::string line; std::getline(lines, line);) {
    std::smatch match;
    if (!std::regex_search(line, match, row)) {
      continue;
    }
    const std::string gpios = match[2], headers = match[3];
    std::vector<int> g, h;
    for (auto it = std::sregex_iterator(gpios.begin(), gpios.end(), number); it != std::sregex_iterator(); ++it) {
      g.push_back(std::stoi(it->str()));
    }
    for (auto it = std::sregex_iterator(headers.begin(), headers.end(), number); it != std::sregex_iterator(); ++it) {
      h.push_back(std::stoi(it->str()));
    }
    ASSERT_EQ(g.size(), h.size()) << "README row: " << line;
    for (std::size_t i = 0; i < g.size(); ++i) {
      ASSERT_TRUE(kHeaderPinOfGpio.count(g[i])) << "README row: " << line;
      EXPECT_EQ(kHeaderPinOfGpio.at(g[i]), h[i]) << "README row: " << line;
      readme_gpios.insert(g[i]);
    }
  }
  std::set<int> schematic_gpios;
  for (const auto & c : kSchematic) {
    schematic_gpios.insert(c.gpio);
  }
  EXPECT_EQ(readme_gpios, schematic_gpios) << "README pinout table lists different GPIOs than the schematic";
}
