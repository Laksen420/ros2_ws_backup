cmake_minimum_required(VERSION 3.8)
project(rfid_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# IMPORTANT: Add the path to your "rfid_ros2" subdirectory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rfid_ros2
)

add_executable(rfid_node
  src/rfid_node.cpp
  src/CAENRFIDLib_Light.c
  src/IO_Light.c
  src/host.c
)

ament_target_dependencies(rfid_node
  rclcpp
  std_msgs
)

install(TARGETS rfid_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
