cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(frenet_conversion REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_filter REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(detection_core src/detection_core.cpp)
target_include_directories(detection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(detect_node src/detect_node.cpp)
target_include_directories(detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(detect_node
  ament_index_cpp
  f110_msgs
  frenet_conversion
  geometry_msgs
  grid_filter
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(detect_node
  detection_core
  frenet_conversion::frenet_conversion_lib
  grid_filter::grid_filter_lib
)

add_executable(detect src/detect_main.cpp)
target_link_libraries(detect detect_node)
ament_target_dependencies(detect rclcpp)

# --- tracking (C++ port of perception/tracking.py, 2026-08-22) ---
add_library(tracking_core src/tracking_core.cpp)
target_include_directories(tracking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tracking_core f110_msgs frenet_conversion)
target_link_libraries(tracking_core
  Eigen3::Eigen
  frenet_conversion::frenet_conversion_lib
)

add_library(tracking_node src/tracking_node.cpp)
target_include_directories(tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tracking_node
  f110_msgs
  frenet_conversion
  nav_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(tracking_node tracking_core)

add_executable(tracking src/tracking_main.cpp)
target_link_libraries(tracking tracking_node)
ament_target_dependencies(tracking rclcpp)

ament_python_install_package(${PROJECT_NAME})

# 2026-08-22: python tracker kept as the legacy fallback (tracker_exec:=tracking_python)
# install(PROGRAMS
#   perception/tracking.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME tracking
# )
install(PROGRAMS
  perception/tracking.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tracking_python
)

install(PROGRAMS
  perception/detect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detect_python
)

install(PROGRAMS
  perception/intensity_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME intensity_filter
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_detection_core test/test_detection_core.cpp)
  target_link_libraries(test_detection_core detection_core)
  ament_add_gtest(test_tracking_core test/test_tracking_core.cpp)
  target_link_libraries(test_tracking_core tracking_core)
  target_compile_definitions(test_tracking_core PRIVATE
    TRACKING_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS detection_core detect_node tracking_core tracking_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS detect tracking
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
