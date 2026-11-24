// One-shot motor stop, run by hardware.launch.py whenever the hardware container exits.
//
// sysfs PWM and claimed GPIO levels outlive the process that set them: after a crash or SIGKILL the bridges keep
// the last duty cycle, direction and STBY level, so the robot would keep driving with nothing in control. This
// claims the TB6612FNG's STBY pin and both channels with the motor driver's own parameters (launch it under the
// name `motor_driver`): STBY low, IN1/IN2 low, duty zero, then releases them with the PWM disabled and STBY low.

#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "autonom_hardware/tb6612_motor.hpp"
#include "autonom_hardware/gpio.hpp"
#include "autonom_hardware/parameter_utils.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  {
    const auto node = std::make_shared<rclcpp::Node>("motor_safe_stop");
    try {
      const auto gpio_chip = autonom_hardware::declareParameter<std::string>(
        *node, "gpio_chip", "pinctrl-rp1", "GPIO chip name, label, path or number for the direction pins");
      const auto standby_pin = autonom_hardware::declareStandbyPin(*node, 26);
      const auto left_config = autonom_hardware::declareTb6612Config(*node, "left_motor", 0, 5, 6);
      const auto right_config = autonom_hardware::declareTb6612Config(*node, "right_motor", 1, 20, 21);
      const auto chip = autonom_hardware::openGpioChip(gpio_chip);
      // STBY low first turns both outputs off; each channel's construction then drives IN1/IN2 low and duty 0,
      // and its destruction disables the PWM. STBY is released last, driven low.
      {
        const autonom_hardware::GpioOutput standby(chip, standby_pin, "autonom_safe_stop_standby", false);
        const autonom_hardware::Tb6612Motor left(chip, left_config, "autonom_safe_stop_left");
        const autonom_hardware::Tb6612Motor right(chip, right_config, "autonom_safe_stop_right");
      }
      RCLCPP_WARN(node->get_logger(), "Hardware process exited: motors stopped and PWM disabled");
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node->get_logger(), "Could not stop the motors: %s. CUT MOTOR POWER.", error.what());
      exit_code = 1;
    }
  }
  rclcpp::shutdown();
  return exit_code;
}
