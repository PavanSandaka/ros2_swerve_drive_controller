#include <gtest/gtest.h>

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadSwerveDriveController, load_controller) {
  // Create a single-threaded executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Create a ControllerManager with a ResourceManager and a test URDF
  controller_manager::ControllerManager cm(
      std::make_unique<hardware_interface::ResourceManager>(
          ros2_control_test_assets::minimal_robot_urdf),
      executor, "test_swerve_controller_manager");

  // Attempt to load the Swerve Drive Controller
  auto controller =
      cm.load_controller("test_swerve_drive_controller",
                         "swerve_drive_controller/SwerveController");

  // Check if the controller was loaded successfully
  ASSERT_NE(controller, nullptr) << "Failed to load Swerve Drive Controller.";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
