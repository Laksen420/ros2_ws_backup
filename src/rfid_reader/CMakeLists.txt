cmake_minimum_required(VERSION 3.8)
project(rfid_reader)

# Find ament_cmake so we can build
find_package(ament_cmake REQUIRED)

# Add the executable and include all source files
add_executable(rfid_node
  src/CAENRFIDLib_Light.c
  src/IO_Light.c
  src/host.c
  src/rfid_node.c
)

# Include header files
target_include_directories(rfid_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rfid_reader
)

# Link any necessary system libraries (e.g., pthread)
target_link_libraries(rfid_node PRIVATE pthread)

# Install the executable
install(TARGETS rfid_node
  DESTINATION lib/${PROJECT_NAME}
)

# Declare ament package
ament_package()
