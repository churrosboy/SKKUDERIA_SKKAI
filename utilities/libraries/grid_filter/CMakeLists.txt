cmake_minimum_required(VERSION 3.8)
project(grid_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(yaml-cpp REQUIRED)

add_library(grid_filter_lib src/grid_filter.cpp)
target_include_directories(grid_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(grid_filter_lib PUBLIC ${OpenCV_LIBS} yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS grid_filter_lib
  EXPORT export_grid_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_filter test/test_grid_filter.cpp)
  target_link_libraries(test_grid_filter grid_filter_lib)
endif()

ament_export_targets(export_grid_filter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV yaml-cpp)
ament_package()
