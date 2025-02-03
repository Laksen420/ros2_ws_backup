cmake_minimum_required(VERSION 3.5)
project(rfid_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories (add both the package include and the sdk folder)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/sdk
)

# Option 1: Build the SDK files together with your node:
add_executable(rfid_node 
  src/rfid_node.cpp 
  sdk/CAENRFIDLib_Light.c 
  sdk/host.c 
  sdk/IO_Light.c
)

ament_target_dependencies(rfid_node rclcpp std_msgs)

# Install targets
install(TARGETS
  rfid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files if needed
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
