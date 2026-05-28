cmake_minimum_required(VERSION 3.15)
project(ecn_2021)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# main source file
add_executable(control control_node.cpp)
ament_target_dependencies(
   control rclcpp sensor_msgs tf2_ros geometry_msgs)
  
install(TARGETS control DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch urdf map DESTINATION share/${PROJECT_NAME})
  
ament_package()