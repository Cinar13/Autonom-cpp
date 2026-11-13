# CMake generated Testfile for 
# Source directory: /home/max/Autonom-cpp/src/autonom_patrol
# Build directory: /home/max/Autonom-cpp/build/autonom_patrol
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_track_planner "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/max/Autonom-cpp/build/autonom_patrol/test_results/autonom_patrol/test_track_planner.gtest.xml" "--package-name" "autonom_patrol" "--output-file" "/home/max/Autonom-cpp/build/autonom_patrol/ament_cmake_gtest/test_track_planner.txt" "--command" "/home/max/Autonom-cpp/build/autonom_patrol/test_track_planner" "--gtest_output=xml:/home/max/Autonom-cpp/build/autonom_patrol/test_results/autonom_patrol/test_track_planner.gtest.xml")
set_tests_properties(test_track_planner PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/max/Autonom-cpp/build/autonom_patrol/test_track_planner" TIMEOUT "60" WORKING_DIRECTORY "/home/max/Autonom-cpp/build/autonom_patrol" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/max/Autonom-cpp/src/autonom_patrol/CMakeLists.txt;57;ament_add_gtest;/home/max/Autonom-cpp/src/autonom_patrol/CMakeLists.txt;0;")
subdirs("gtest")
