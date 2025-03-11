#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller {

SwerveKinematics::SwerveKinematics()
    : m_frontLocation(0.0_m, 0.0_m), m_backLeftLocation(0.0_m, 0.0_m),
      m_backRightLocation(0.0_m, 0.0_m),
      m_kinematics(m_frontLocation, m_backLeftLocation, m_backRightLocation) {}

void SwerveKinematics::initializeKinematics(double front_x, double front_y,
                                            double back_left_x,
                                            double back_left_y,
                                            double back_right_x,
                                            double back_right_y) {
  m_frontLocation =
      frc::Translation2d(units::meter_t(front_x), units::meter_t(front_y));
  m_backLeftLocation = frc::Translation2d(units::meter_t(back_left_x),
                                          units::meter_t(back_left_y));
  m_backRightLocation = frc::Translation2d(units::meter_t(back_right_x),
                                           units::meter_t(back_right_y));

  m_kinematics = frc::SwerveDriveKinematics<3>(
      m_frontLocation, m_backLeftLocation, m_backRightLocation);
}

SwerveModuleStates
SwerveKinematics::ComputeStates(double linear_x_cmd, double linear_y_cmd,
                                double angular_cmd,
                                const std::vector<double> &joint_distances) {
  frc::ChassisSpeeds speeds{units::meters_per_second_t(linear_x_cmd),
                            units::meters_per_second_t(linear_y_cmd),
                            units::radians_per_second_t(angular_cmd)};

  // Compute module states
  auto [front, backLeft, backRight] = m_kinematics.ToSwerveModuleStates(speeds);

  if (joint_distances.size() < 6) {
    // RCLCPP_WARN(logger, "Insufficient joint distances provided!");
    return SwerveModuleStates{front, backLeft, backRight};
  }

  auto frontOptimized = frc::SwerveModuleState::Optimize(
      front, units::radian_t(joint_distances[1]));
  auto backLeftOptimized = frc::SwerveModuleState::Optimize(
      backLeft, units::radian_t(joint_distances[3]));
  auto backRightOptimized = frc::SwerveModuleState::Optimize(
      backRight, units::radian_t(joint_distances[5]));

  return SwerveModuleStates{
      frontOptimized,    // front module state
      backLeftOptimized, // back left module state
      backRightOptimized // back right module state
  };
}
} // namespace swerve_drive_controller
