cmake_minimum_required(VERSION 3.8)
project(balltze_hardware)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(dynamixel_sdk REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  dynamixel_sdk
  rclcpp
  rclcpp_lifecycle
  pluginlib
  controller_manager
)

include_directories(include)

add_library(balltze_hardware_plugins SHARED
  src/ax_12_a_motor.cpp
  src/balltze_dynamixel_system.cpp
)

target_include_directories(balltze_hardware_plugins PRIVATE
  include
)

ament_target_dependencies(balltze_hardware_plugins
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(balltze_hardware_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface balltze_hardware_plugins.xml)

install(
  TARGETS balltze_hardware_plugins
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES balltze_hardware_plugins.xml
  DESTINATION share/balltze_hardware_plugins
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(balltze_hardware_plugins)

ament_export_include_directories(
  include
)

ament_export_libraries(
  balltze_hardware_plugins
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()