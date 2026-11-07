cmake_minimum_required(VERSION 3.16)
project(autonom_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED
  src/odometry_covariance_relay.cpp
  src/scan_decimator.cpp
)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_localization::OdometryCovarianceRelay"
  EXECUTABLE odometry_covariance_relay
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_localization::ScanDecimator"
  EXECUTABLE scan_decimator
)

install(TARGETS ${PROJECT_NAME}_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
