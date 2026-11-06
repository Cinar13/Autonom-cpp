cmake_minimum_required(VERSION 3.16)
project(robot_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds models DESTINATION share/${PROJECT_NAME})

ament_package()
