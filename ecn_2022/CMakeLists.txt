cmake_minimum_required(VERSION 3.5)
project(ecn_2022)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# main source file
ament_auto_add_executable(control_node control_node.cpp)
target_include_directories(control_node PRIVATE include ${Eigen3_INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)