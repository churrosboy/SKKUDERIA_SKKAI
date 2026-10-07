cmake_minimum_required(VERSION 3.8)
project(spline_planner)

# 2026-08-22: converted from ament_python to ament_cmake for the C++ port of
# spline_planner.py. The python node is still installed as `spline_planner_python`.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(spliner_core
  src/cubic_spline.cpp
  src/py_frenet_converter.cpp
  src/spliner_core.cpp
)
target_include_directories(spliner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(spliner_core f110_msgs)

add_library(spliner_node src/spliner_node.cpp)
target_include_directories(spliner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(spliner_node
  f110_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  std_msgs
  visualization_msgs
)
target_link_libraries(spliner_node spliner_core)

add_executable(spline_planner src/spliner_main.cpp)
target_link_libraries(spline_planner spliner_node)
ament_target_dependencies(spline_planner rclcpp)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  spline_planner/spline_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spline_planner_python
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS spliner_core spliner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS spline_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_spliner_core test/test_spliner_core.cpp)
  target_link_libraries(test_spliner_core spliner_core)
  target_compile_definitions(test_spliner_core PRIVATE
    SPLINER_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
endif()

ament_package()
