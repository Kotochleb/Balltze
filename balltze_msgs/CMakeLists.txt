cmake_minimum_required(VERSION 3.5)

project(balltze_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_executable(
  message_relay_node
  src/message_relay.cpp
  src/message_relay_node.cpp
)

ament_target_dependencies(message_relay_node
rclcpp
sensor_msgs
trajectory_msgs
)

install(TARGETS message_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

set(msg_files
  "msg/MotorStatus.msg"
  "msg/MotorStatusArr.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs trajectory_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()