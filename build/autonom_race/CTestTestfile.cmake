# CMake generated Testfile for 
# Source directory: /home/max/Autonom-cpp/src/autonom_race
# Build directory: /home/max/Autonom-cpp/build/autonom_race
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_start_gate "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/max/Autonom-cpp/build/autonom_race/test_results/autonom_race/test_start_gate.gtest.xml" "--package-name" "autonom_race" "--output-file" "/home/max/Autonom-cpp/build/autonom_race/ament_cmake_gtest/test_start_gate.txt" "--command" "/home/max/Autonom-cpp/build/autonom_race/test_start_gate" "--gtest_output=xml:/home/max/Autonom-cpp/build/autonom_race/test_results/autonom_race/test_start_gate.gtest.xml")
set_tests_properties(test_start_gate PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/max/Autonom-cpp/build/autonom_race/test_start_gate" TIMEOUT "60" WORKING_DIRECTORY "/home/max/Autonom-cpp/build/autonom_race" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/max/Autonom-cpp/src/autonom_race/CMakeLists.txt;46;ament_add_gtest;/home/max/Autonom-cpp/src/autonom_race/CMakeLists.txt;0;")
subdirs("gtest")
