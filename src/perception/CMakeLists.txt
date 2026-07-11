cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(msgs REQUIRED)

include_directories(include)

add_executable(vision_detector_node src/vision_detector_node.cpp)
ament_target_dependencies(vision_detector_node
  rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV msgs)
target_link_libraries(vision_detector_node ${OpenCV_LIBRARIES})

install(TARGETS vision_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
