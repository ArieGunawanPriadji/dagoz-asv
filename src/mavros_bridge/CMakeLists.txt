cmake_minimum_required(VERSION 3.8)
project(mavros_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msgs REQUIRED)

include_directories(include)

add_executable(mavros_bridge_node src/mavros_bridge_node.cpp)
ament_target_dependencies(mavros_bridge_node
  rclcpp mavros_msgs sensor_msgs msgs)

install(TARGETS mavros_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
