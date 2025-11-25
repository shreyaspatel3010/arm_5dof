cmake_minimum_required(VERSION 3.8)
project(arm_teensy_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(arm_teensy_hardware SHARED
  src/arm_teensy_system.cpp
)

target_include_directories(arm_teensy_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_teensy_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface arm_teensy_hardware_plugins.xml)

install(TARGETS arm_teensy_hardware
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(arm_teensy_hardware)
ament_export_include_directories(include)

ament_package()
