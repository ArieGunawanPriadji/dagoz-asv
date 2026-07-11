cmake_minimum_required(VERSION 3.8)
project(asv_guidance_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(asv_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_follower_node src/waypoint_follower_node.cpp)
ament_target_dependencies(waypoint_follower_node
  rclcpp sensor_msgs geometry_msgs asv_msgs)

install(TARGETS waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
