cmake_minimum_required(VERSION 3.16)
project(autonom_patrol LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/track_planner.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(${PROJECT_NAME}_nodes SHARED src/loop_patrol_node.cpp)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "autonom_patrol::LoopPatrolNode"
  EXECUTABLE loop_patrol
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_track_planner test/test_track_planner.cpp)
  target_link_libraries(test_track_planner ${PROJECT_NAME}_core)
endif()

ament_package()
