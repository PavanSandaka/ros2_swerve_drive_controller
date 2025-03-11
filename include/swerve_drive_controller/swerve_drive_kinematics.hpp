#ifndef SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_KINEMATICS_HPP_

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

namespace swerve_drive_controller {

struct SwerveModuleStates {
  frc::SwerveModuleState front;
  frc::SwerveModuleState backLeft;
  frc::SwerveModuleState backRight;
};

class SwerveKinematics {
public:
  SwerveKinematics();

  void initializeKinematics(double front_x, double front_y, double back_left_x,
                            double back_left_y, double back_right_x,
                            double back_right_y);

  SwerveModuleStates ComputeStates(double linear_x_cmd, double linear_y_cmd,
                                   double angular_cmd,
                                   const std::vector<double> &joint_distances);

  const frc::SwerveDriveKinematics<3> &getKinematics() const {
    return m_kinematics;
  }

private:
  frc::Translation2d m_frontLocation;
  frc::Translation2d m_backLeftLocation;
  frc::Translation2d m_backRightLocation;
  frc::SwerveDriveKinematics<3> m_kinematics;
};

} // namespace swerve_drive_controller
#endif // SWERVE_DRIVE_KINEMATICS_HPP_
