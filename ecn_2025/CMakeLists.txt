cmake_minimum_required(VERSION 3.15)
project(ecn_2025)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(VISP REQUIRED COMPONENTS core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# main source file
ament_auto_add_executable(control control_node.cpp src/visual_servo.cpp)
target_include_directories(control PRIVATE include ${VISP_INCLUDE_DIRS})
target_link_libraries(control ${VISP_LIBRARIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS src/ball.py DESTINATION lib/${PROJECT_NAME})
  
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)