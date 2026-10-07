# Standalone build for the GL5 ROS2 driver node (see gl5_node.cpp).
# Build with ROS2 Humble sourced:
#   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
# Links against the SDK built in ../_archive_ (run the top-level SDK build first).
cmake_minimum_required(VERSION 3.10)
project(soslab_gl5_driver_sdk)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(SDK_LIB "${SDK_ROOT}/_archive_/lib/libLidar_x64_release.so")

add_executable(soslab_gl5_driver_node gl5_node.cpp)
target_include_directories(soslab_gl5_driver_node PRIVATE
  "${SDK_ROOT}/_archive_/include"
  "${SDK_ROOT}/_archive_/include/nlohmann_json"
)
target_link_libraries(soslab_gl5_driver_node ${SDK_LIB} rclcpp::rclcpp ${sensor_msgs_TARGETS})

# Same RUNPATH scheme as the original prebuilt package: the .so ships alongside
# the binary (prebuilt/) or one level up (install lib/), ROS libs from /opt.
set_target_properties(soslab_gl5_driver_node PROPERTIES
  BUILD_RPATH "\$ORIGIN:\$ORIGIN/..:/opt/ros/humble/lib"
  INSTALL_RPATH "\$ORIGIN:\$ORIGIN/..:/opt/ros/humble/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)
