#ifndef SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER_HPP_

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <chrono>
#include <cmath>
#include <hardware_interface/loaned_command_interface.hpp>
#include <memory>
#include <queue>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace swerve_drive_controller {

using CallbackReturn = controller_interface::CallbackReturn;

class SwerveHardware {
public:
  using CommandInterfaceRef =
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>;

  void addWheelHandle(const std::string &joint_name,
                      CommandInterfaceRef command_interface);
  void addAxleHandle(const std::string &joint_name,
                     CommandInterfaceRef command_interface);

  CommandInterfaceRef getWheelHandle(const std::string &joint_name);
  CommandInterfaceRef getAxleHandle(const std::string &joint_name);

  void setPosition(const std::string &axle_name, double position);
  void setSpeed(const std::string &wheel_name, double speed);

private:
  std::unordered_map<std::string, CommandInterfaceRef> wheel_handles_;
  std::unordered_map<std::string, CommandInterfaceRef> axle_handles_;
};

class SwerveController : public controller_interface::ControllerInterface {
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  SwerveController();

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  std::shared_ptr<SwerveHardware>
  create_swerve_hardware(const std::vector<std::string> &wheel_joints,
                         const std::vector<std::string> &axle_joints);

protected:
  std::shared_ptr<SwerveHardware> swerve_hardware_handle_;
  std::vector<std::string> wheel_joints_;
  std::vector<std::string> axle_joints_;

  std::string odometry_topic_;
  std::string base_footprint_;

  frc::Pose2d m_pose;            // Current pose of the robot
  double last_gyro_angle_ = 0.0; // Store the last gyro angle (yaw)
  frc::Rotation2d rotation{
      units::radian_t(last_gyro_angle_)}; // Rotation object

  std::unique_ptr<swerve_drive_controller::SwerveKinematics> swerve_kinematics_;

  std::unique_ptr<frc::SwerveDriveOdometry<3>> m_odometry;

  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;

  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

  struct WheelParams {
    double x_offset = 0.0; // Chassis Center to Axle Center
    double y_offset = 0.0; // Axle Center to Wheel Center
    double radius = 0.0;   // Assumed to be the same for all wheels
  } wheel_params_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};
  rclcpp::Time previous_update_timestamp_{0};

  // Topic Subscription
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>>
      received_velocity_msg_ptr_{nullptr};
  realtime_tools::RealtimeBox<std::shared_ptr<sensor_msgs::msg::JointState>>
      received_joint_state_msg_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_ =
      nullptr; // IMU subscription
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_subscription_ = nullptr; // JointStates subscription

  sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;

  std::vector<std::string> joint_names_; // List of joint names
  std::unordered_map<std::string, double> initial_joint_positions_;

  frc::Rotation2d gyroAngle;

  // Publishers
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>
      odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
      realtime_odometry_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>>
      odometry_transform_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
      realtime_odometry_transform_publisher_ = nullptr;

  bool is_halted_ = false;
  bool use_stamped_vel_ = true;
  bool reset();
  void halt();
};

} // namespace swerve_drive_controller
#endif // SWERVE_DRIVE_CONTROLLER_HPP_
