# CMake generated Testfile for 
# Source directory: /home/max/Autonom-cpp/src/autonom_localization
# Build directory: /home/max/Autonom-cpp/build/autonom_localization
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_speckle_filter "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/max/Autonom-cpp/build/autonom_localization/test_results/autonom_localization/test_speckle_filter.gtest.xml" "--package-name" "autonom_localization" "--output-file" "/home/max/Autonom-cpp/build/autonom_localization/ament_cmake_gtest/test_speckle_filter.txt" "--command" "/home/max/Autonom-cpp/build/autonom_localization/test_speckle_filter" "--gtest_output=xml:/home/max/Autonom-cpp/build/autonom_localization/test_results/autonom_localization/test_speckle_filter.gtest.xml")
set_tests_properties(test_speckle_filter PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/max/Autonom-cpp/build/autonom_localization/test_speckle_filter" TIMEOUT "60" WORKING_DIRECTORY "/home/max/Autonom-cpp/build/autonom_localization" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;95;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/max/Autonom-cpp/src/autonom_localization/CMakeLists.txt;65;ament_add_gtest;/home/max/Autonom-cpp/src/autonom_localization/CMakeLists.txt;0;")
subdirs("gtest")
