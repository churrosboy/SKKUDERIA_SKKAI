cmake_minimum_required(VERSION 3.8)
project(soslab_gl5_driver)

find_package(ament_cmake REQUIRED)

# Prebuilt driver binary (source lost — see package.xml). RUNPATH is
# $ORIGIN:$ORIGIN/..:/opt/ros/humble/lib, so libLidar_x64_release.so is found
# both under --symlink-install (lib beside binary in prebuilt/) and a regular
# install (binary in lib/${PROJECT_NAME}/, lib in lib/).
install(PROGRAMS prebuilt/soslab_gl5_driver_node
  DESTINATION lib/${PROJECT_NAME})
install(FILES prebuilt/libLidar_x64_release.so
  DESTINATION lib)

ament_package()
