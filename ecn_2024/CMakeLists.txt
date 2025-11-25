cmake_minimum_required(VERSION 3.5)
project(ecn_2024)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
# find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Letter.srv" DEPENDENCIES geometry_msgs)

# main source file
ament_auto_add_executable(control control_node.cpp)
target_include_directories(control PRIVATE include)

if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
    rosidl_target_interfaces(control ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    target_link_libraries(control "${cpp_typesupport_target}")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS src/letter2path control_node.py DESTINATION lib/${PROJECT_NAME})

install(FILES src/tracker.py DESTINATION share/${PROJECT_NAME})
  
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)