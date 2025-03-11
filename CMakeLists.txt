cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)  # Added for testing
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(wpilib REQUIRED)

pluginlib_export_plugin_description_file(controller_interface swerve_drive_plugin.xml)

# Define the main library
add_library(swerve_drive_controller SHARED 
  src/swerve_drive_controller.cpp
  src/swerve_drive_kinematics.cpp
)

add_subdirectory(test)

target_include_directories(swerve_drive_controller PRIVATE include)

# Link libraries
target_link_libraries(swerve_drive_controller 
  wpiutil 
  wpilibc 
  wpimath
)

ament_target_dependencies(swerve_drive_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)

# Install the library and plugin description
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS swerve_drive_controller
  EXPORT export_swerve_drive_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES swerve_drive_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(swerve_drive_controller)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
