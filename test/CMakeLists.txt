cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

ament_add_gtest(test_load_swerve_drive_controller test_load_swerve_drive_controller.cpp)

ament_target_dependencies(test_load_swerve_drive_controller
  controller_manager
  hardware_interface
  ros2_control_test_assets
  rclcpp
)

target_include_directories(test_load_swerve_drive_controller PRIVATE
  include
)

target_link_libraries(test_load_swerve_drive_controller
  swerve_drive_controller
  rclcpp::rclcpp
)

install(TARGETS test_load_swerve_drive_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
