cmake_minimum_required(VERSION 3.16)
project(autonom_competitor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/lane_follower.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(${PROJECT_NAME}_nodes SHARED src/competitor_driver_node.cpp)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_competitor::CompetitorDriverNode"
  EXECUTABLE competitor_driver
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane_follower test/test_lane_follower.cpp)
  target_link_libraries(test_lane_follower ${PROJECT_NAME}_core)
endif()

ament_package()
