cmake_minimum_required(VERSION 3.8)
project(controller)

# ament_cmake conversion for the C++ PP controller port (2026-08-21), mirroring
# the perception package's detection migration. The python node stays installed
# as exec `controller` (the default in controller_launch.xml); the C++ node is
# the opt-in exec `controller_cpp` (ctrl_exec:=controller_cpp).

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(frenet_conversion REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# pure PP logic, no ROS deps -- unit-tested against python golden vectors
add_library(pp_core src/pp_core.cpp)
target_include_directories(pp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# pure FTG fallback logic (port of controller/ftg.py), no ROS deps
add_library(ftg_core src/ftg_core.cpp)
target_include_directories(ftg_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(controller_node
  ament_index_cpp
  ackermann_msgs
  f110_msgs
  frenet_conversion
  geometry_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(controller_node
  pp_core
  ftg_core
  frenet_conversion::frenet_conversion_lib
  yaml-cpp
)

add_executable(controller_cpp src/controller_main.cpp)
target_link_libraries(controller_cpp controller_node)
ament_target_dependencies(controller_cpp rclcpp)

# keep the python package importable + the python node runnable as `controller`
# (was a setup.py console_script; install(PROGRAMS) needs the shebang + main
# guard added to controller_manager.py)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  controller/controller_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pp_core test/test_pp_core.cpp)
  target_include_directories(test_pp_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(test_pp_core pp_core)
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS pp_core ftg_core controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS controller_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
