cmake_minimum_required(VERSION 3.8)
project(frenet_conversion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(f110_msgs REQUIRED)

add_library(frenet_conversion_lib src/frenet_converter.cpp)
target_include_directories(frenet_conversion_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(frenet_conversion_lib f110_msgs)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  frenet_conversion/frenet_converter_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frenet_converter_demo
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS frenet_conversion_lib
  EXPORT export_frenet_conversion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frenet_converter test/test_frenet_converter.cpp)
  target_link_libraries(test_frenet_converter frenet_conversion_lib)
endif()

ament_export_targets(export_frenet_conversion HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(f110_msgs)
ament_package()
