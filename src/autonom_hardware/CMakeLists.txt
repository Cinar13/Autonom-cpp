cmake_minimum_required(VERSION 3.16)
project(autonom_hardware LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET libgpiod>=1.6)
if(GPIOD_VERSION VERSION_GREATER_EQUAL 2.0)
  message(FATAL_ERROR "libgpiod ${GPIOD_VERSION} found; this package targets the v1.6 API shipped with Ubuntu 24.04")
endif()

add_library(${PROJECT_NAME}_core SHARED
  src/gpio.cpp
  src/pwm_channel.cpp
  src/tb6612_motor.cpp
  src/quadrature_encoder.cpp
  src/pid_controller.cpp
  src/diff_drive_kinematics.cpp
  src/encoder_model.cpp
  src/button_debouncer.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC PkgConfig::GPIOD pthread)

add_library(${PROJECT_NAME}_nodes SHARED
  src/motor_driver_node.cpp
  src/wheel_odometry_node.cpp
  src/sim_wheel_odometry_node.cpp
  src/start_button_node.cpp
)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2_ros::tf2_ros
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_hardware::MotorDriverNode"
  EXECUTABLE motor_driver_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_hardware::WheelOdometryNode"
  EXECUTABLE wheel_odometry_node
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_hardware::SimWheelOdometryNode"
  EXECUTABLE sim_wheel_odometry_node
  PLUGIN "autonom_hardware::StartButtonNode"
  EXECUTABLE start_button_node
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_diff_drive_kinematics test/test_diff_drive_kinematics.cpp)
  target_link_libraries(test_diff_drive_kinematics ${PROJECT_NAME}_core)
  ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
  target_link_libraries(test_pid_controller ${PROJECT_NAME}_core)
  ament_add_gtest(test_encoder_model test/test_encoder_model.cpp)
  target_link_libraries(test_encoder_model ${PROJECT_NAME}_core)
  ament_add_gtest(test_button_debouncer test/test_button_debouncer.cpp)
  target_link_libraries(test_button_debouncer ${PROJECT_NAME}_core)

  # Same sources linked against test/fakes/fake_gpiod.cpp instead of libgpiod, so GPIO-facing code runs here
  # without a Pi (or root for gpio-sim). PWM tests point the sysfs path at a temporary directory.
  add_library(${PROJECT_NAME}_core_fake STATIC
    src/gpio.cpp
    src/pwm_channel.cpp
    src/tb6612_motor.cpp
    src/quadrature_encoder.cpp
    src/pid_controller.cpp
    src/diff_drive_kinematics.cpp
    src/button_debouncer.cpp
    src/encoder_model.cpp
    test/fakes/fake_gpiod.cpp
  )
  target_include_directories(${PROJECT_NAME}_core_fake PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fakes
    ${GPIOD_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_core_fake PUBLIC pthread)

  foreach(test_name test_gpio test_pwm_channel test_tb6612_motor test_quadrature_encoder)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 60)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core_fake)
  endforeach()

  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(test_pinout test/test_pinout.cpp)
  target_compile_definitions(test_pinout PRIVATE
    HARDWARE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/hardware.yaml"
    SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_pinout yaml-cpp::yaml-cpp)

  ament_add_gtest(test_hardware_nodes
    test/test_hardware_nodes.cpp
    src/motor_driver_node.cpp
    src/wheel_odometry_node.cpp
    src/start_button_node.cpp
    TIMEOUT 120
  )
  target_compile_definitions(test_hardware_nodes PRIVATE
    HARDWARE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/config/hardware.yaml")
  target_link_libraries(test_hardware_nodes
    ${PROJECT_NAME}_core_fake
    rclcpp::rclcpp
    rclcpp_components::component
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    tf2_ros::tf2_ros
  )
endif()

ament_package()
