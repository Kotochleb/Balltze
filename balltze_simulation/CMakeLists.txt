cmake_minimum_required(VERSION 3.10.2)
project(balltze_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

include_directories(
  include
  ${CMAKE_BINARY_DIR}
)

add_executable(
  tip_sensor_test
  src/tip_sensor_test.cpp
  src/gz_tip.cpp
)

ament_target_dependencies(tip_sensor_test
  ament_cmake
  ament_index_cpp
  rclcpp
)

target_link_libraries(tip_sensor_test
  ignition-transport${IGN_TRANSPORT_VER}::core
)

install(TARGETS tip_sensor_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()