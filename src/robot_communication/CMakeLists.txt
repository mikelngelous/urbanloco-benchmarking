cmake_minimum_required(VERSION 3.16)
project(robot_communication)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for performance
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_filters
)

# Find system dependencies
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Add message files
add_message_files(
  FILES
  SensorPacket.msg
)

# Add service files
add_service_files(
  FILES
  ConfigUpdate.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Generate protobuf files
set(PROTO_FILES proto/sensor_data.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Create custom target for protobuf generation
add_custom_target(generate_protobuf_files DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})

# Ensure proto files are generated before building library
add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
       --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}/proto/sensor_data.proto
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/proto/sensor_data.proto
  COMMENT "Compiling protobuf sensor_data.proto"
  VERBATIM
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # For generated protobuf headers
)

# Catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime message_filters
  DEPENDS Boost Protobuf
)

# Core library
add_library(${PROJECT_NAME}
  src/sensor_subscriber.cpp
  src/communication_manager.cpp
  src/protocol_handler.cpp
  src/sensor_fusion.cpp
  src/hybrid_synchronizer.cpp
  src/data_transfer_queue.cpp
  src/camera_timestamp_rectifier.cpp
  src/utils/thread_pool.cpp
  ${PROTO_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${LZ4_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_protobuf_files)

# Robot data publisher node
add_executable(robot_data_publisher
  src/nodes/robot_data_publisher.cpp
)

target_link_libraries(robot_data_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(robot_data_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Mission-server node
add_executable(mission_server
  src/nodes/mission_server.cpp
)

target_link_libraries(mission_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(mission_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install targets
install(TARGETS ${PROJECT_NAME} robot_data_publisher mission_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)


# Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  
  # Unit tests
  catkin_add_gtest(${PROJECT_NAME}_unit_tests
    tests/unit_tests/test_minimal_functionality.cpp
  )
  
  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME})
  endif()
  
  # Integration tests
  add_rostest_gtest(${PROJECT_NAME}_integration_tests
    tests/integration_tests/test_end_to_end.launch
    tests/integration_tests/test_end_to_end.cpp
  )
  
  if(TARGET ${PROJECT_NAME}_integration_tests)
    target_link_libraries(${PROJECT_NAME}_integration_tests ${PROJECT_NAME})
  endif()
endif()
