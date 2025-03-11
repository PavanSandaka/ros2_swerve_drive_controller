#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <cmath>
#include <memory>
#include <queue>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace {

// constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel_stamped";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_IMU_TOPIC = "/imu";
constexpr auto DEFAULT_JOINT_STATES_TOPIC = "/joint_states";
constexpr auto DEFAULT_IGN_JOINT_STATES_TOPIC = "/ign/joint_states";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

} // namespace

namespace swerve_drive_controller {

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SwerveController::SwerveController()
    : controller_interface::ControllerInterface() {
  auto zero_twist = std::make_shared<Twist>();
  zero_twist->header.stamp = rclcpp::Time(0);
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.set(zero_twist);
}

CallbackReturn SwerveController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_init...");

  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>(
        "wheel_joints", {"front_wheel_joint", "rear_left_wheel_joint",
                         "rear_right_wheel_joint"});

    auto_declare<std::vector<std::string>>(
        "axle_joints",
        {"front_axle_joint", "rear_left_axle_joint", "rear_right_axle_joint"});

    auto_declare<double>("chassis_length", wheel_params_.x_offset);
    auto_declare<double>("chassis_width", wheel_params_.y_offset);
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    auto_declare<std::string>("odom", odometry_topic_);
    auto_declare<std::string>("base_footprint", base_footprint_);
    auto_declare<double>("publish_rate", publish_rate_);

    RCLCPP_INFO(get_node()->get_logger(),
                "Command Interfaces Length In on_init: %ld",
                command_interfaces_.size());
  }

  catch (const std::exception &e) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Exception thrown during init stage with message: %s \n",
                e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration
SwerveController::command_interface_configuration() const {
  RCLCPP_INFO(
      get_node()->get_logger(),
      "[SWERVE_DRIVE_CONTROLLER] INSIDE command_interface_configuration...");

  std::vector<std::string> conf_names;

  // Add velocity interfaces for all wheel joints
  for (const auto &wheel_joint : wheel_joints_) {
    conf_names.push_back(wheel_joint + "/" + HW_IF_VELOCITY);
  }

  // Add position interfaces for all axle joints
  for (const auto &axle_joint : axle_joints_) {
    conf_names.push_back(axle_joint + "/" + HW_IF_POSITION);
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration SwerveController::state_interface_configuration() const {
  RCLCPP_INFO(
      get_node()->get_logger(),
      "[SWERVE_DRIVE_CONTROLLER] INSIDE state_interface_configuration...");
  return {interface_configuration_type::NONE};
}

CallbackReturn SwerveController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto logger = get_node()->get_logger();

  RCLCPP_INFO(logger, "[SWERVE_DRIVE_CONTROLLER] INSIDE on_configure...");

  try {
    // TODO: Fetch wheel locations from TF using URDF instead of defining
    // separate parameters.
    if (get_node()->has_parameter("wheel_locations.x") &&
        get_node()->has_parameter("wheel_locations.y")) {
      auto x_locations =
          get_node()->get_parameter("wheel_locations.x").as_double_array();
      auto y_locations =
          get_node()->get_parameter("wheel_locations.y").as_double_array();

      // Ensure the sizes of x and y locations match
      if (x_locations.size() == y_locations.size()) {
        if (x_locations.size() == 3) {
          swerve_kinematics_ = std::make_unique<SwerveKinematics>();

          // Initialize kinematics with the loaded values
          swerve_kinematics_->initializeKinematics(
              x_locations[0], y_locations[0], x_locations[1], y_locations[1],
              x_locations[2], y_locations[2]);
          RCLCPP_INFO(logger, "Swerve kinematics initialized successfully.");

          // Odometry
          frc::Rotation2d initial_rotation{units::radian_t(0.0)};
          m_odometry = std::make_unique<frc::SwerveDriveOdometry<3>>(
              swerve_kinematics_->getKinematics(), initial_rotation,
              std::array<frc::SwerveModulePosition, 3>{
                  frc::SwerveModulePosition{}, frc::SwerveModulePosition{},
                  frc::SwerveModulePosition{}});
          m_pose = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad));
        } else {
          RCLCPP_ERROR(logger, "wheel_locations.x and wheel_locations.y must "
                               "each have exactly 3 "
                               "elements.");
          return CallbackReturn::ERROR;
        }
      } else {
        RCLCPP_ERROR(
            logger,
            "Mismatch between number of x and y locations in wheel_locations.");
      }
    } else {
      RCLCPP_ERROR(
          logger,
          "wheel_locations.x or wheel_locations.y parameter not found.");
    }

    // Retrieve the wheel_joints and axle_joints parameters
    auto wheel_joints =
        get_node()->get_parameter("wheel_joints").as_string_array();
    auto axle_joints =
        get_node()->get_parameter("axle_joints").as_string_array();

    // Validate the retrieved parameters
    if (wheel_joints.size() != axle_joints.size()) {
      RCLCPP_ERROR(
          logger,
          "Mismatch in the size of wheel_joints (%zu) and axle_joints (%zu).",
          wheel_joints.size(), axle_joints.size());
      return CallbackReturn::ERROR;
    }

    if (wheel_joints.empty() || axle_joints.empty()) {
      RCLCPP_ERROR(logger, "wheel_joints or axle_joints cannot be empty.");
      return CallbackReturn::ERROR;
    }

    // Log the retrieved joint names for debugging
    RCLCPP_INFO(logger, "Wheel Joints: [%s]",
                rcpputils::join(wheel_joints, ", ").c_str());
    RCLCPP_INFO(logger, "Axle Joints: [%s]",
                rcpputils::join(axle_joints, ", ").c_str());

    // Store the joint names in class members
    wheel_joints_ = wheel_joints;
    axle_joints_ = axle_joints;

    joint_names_.clear(); // Ensure it's empty before populating

    for (size_t i = 0; i < wheel_joints_.size(); ++i) {
      joint_names_.push_back(wheel_joints_[i]); // Add wheel joint
      joint_names_.push_back(axle_joints_[i]);  // Add corresponding axle joint
    }

    RCLCPP_INFO(logger, "Joint Names for Processing: [%s]",
                rcpputils::join(joint_names_, ", ").c_str());

    wheel_params_.x_offset =
        get_node()->get_parameter("chassis_length").as_double();
    wheel_params_.y_offset =
        get_node()->get_parameter("chassis_width").as_double();
    wheel_params_.radius =
        get_node()->get_parameter("wheel_radius").as_double();

    odometry_topic_ = get_node()->get_parameter("odom").as_string();
    base_footprint_ = get_node()->get_parameter("base_footprint").as_string();
    publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

    int covariance_size = get_node()->get_parameter("covariance_size").as_int();
    auto param_pose_vector =
        get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
    auto param_twist_vector = get_node()
                                  ->get_parameter("twist_covariance_diagonal")
                                  .as_double_array();

    RCLCPP_INFO(logger, "Covariance Size: %d", covariance_size);
    RCLCPP_INFO(logger, "Pose Covariance Diagonal: ");
    for (const auto &val : param_pose_vector) {
      RCLCPP_INFO(logger, "%f", val);
    }
    RCLCPP_INFO(logger, "Twist Covariance Diagonal: ");
    for (const auto &val : param_twist_vector) {
      RCLCPP_INFO(logger, "%f", val);
    }

    pose_covariance_diagonal_.resize(param_pose_vector.size());
    twist_covariance_diagonal_.resize(param_twist_vector.size());

    // Ensure the covariance size matches the expected number of elements
    if (param_pose_vector.size() == covariance_size) {
      for (std::size_t i = 0; i < covariance_size; ++i) {
        pose_covariance_diagonal_[i] = param_pose_vector[i];
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "pose_covariance_diagonal parameter size mismatch");
      return CallbackReturn::ERROR;
    }

    if (param_twist_vector.size() == covariance_size) {
      for (std::size_t i = 0; i < covariance_size; ++i) {
        twist_covariance_diagonal_[i] = param_twist_vector[i];
      }
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "twist_covariance_diagonal parameter size mismatch");
      return CallbackReturn::ERROR;
    }

    cmd_vel_timeout_ = std::chrono::milliseconds(static_cast<int>(
        get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0));

    use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

    if (!reset()) {
      return CallbackReturn::ERROR;
    }

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    // Initialize subscription for velocity command
    if (use_stamped_vel_) {
      velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
          DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<Twist> msg) -> void {
            if (!subscriber_is_active_) {
              RCLCPP_WARN(get_node()->get_logger(),
                          "Can't accept new commands. subscriber is inactive");
              return;
            }

            if ((msg->header.stamp.sec == 0) &&
                (msg->header.stamp.nanosec == 0)) {
              RCLCPP_WARN_ONCE(
                  get_node()->get_logger(),
                  "Received TwistStamped with zero timestamp, setting it to "
                  "current "
                  "time, this message will only be shown once");
              msg->header.stamp = get_node()->get_clock()->now();
            }
            received_velocity_msg_ptr_.set(std::move(msg));
          });
      RCLCPP_INFO(logger, "INSIDE STAMPED TWIST SUBSCRIPTION");
    }

    else {
      velocity_command_unstamped_subscriber_ =
          get_node()->create_subscription<geometry_msgs::msg::Twist>(
              DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
              [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg)
                  -> void {
                if (!subscriber_is_active_) {
                  RCLCPP_WARN(
                      get_node()->get_logger(),
                      "Can't accept new commands. subscriber is inactive");
                  return;
                }
                // Write fake header in the stored stamped command
                std::shared_ptr<Twist> twist_stamped;
                received_velocity_msg_ptr_.get(twist_stamped);
                twist_stamped->twist = *msg;
                twist_stamped->header.stamp = get_node()->get_clock()->now();
              });
      RCLCPP_INFO(logger, "INSIDE UNSTAMPED TWIST SUBSCRIPTION");
    }

    imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        DEFAULT_IMU_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<sensor_msgs::msg::Imu> msg) -> void {
          // Extract the yaw (gyro angle) from the IMU message
          if (msg) {
            tf2::Quaternion quat(msg->orientation.x, msg->orientation.y,
                                 msg->orientation.z, msg->orientation.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            // Store the yaw (gyro angle)
            last_gyro_angle_ = yaw; // Store the latest yaw angle
          }
        });

    // Subscribe to /joint_states topic
    joint_states_subscription_ =
        get_node()->create_subscription<sensor_msgs::msg::JointState>(
            DEFAULT_JOINT_STATES_TOPIC,  // Topic name
            rclcpp::SystemDefaultsQoS(), // QoS settings
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) -> void {
              if (msg) {
                last_joint_state_msg_ = msg;

                if (initial_joint_positions_.empty()) {
                  // Store the initial positions of the joints
                  for (size_t i = 0; i < msg->name.size(); ++i) {
                    const std::string &joint_name = msg->name[i];
                    const double position = msg->position[i];

                    // Store the initial position of the joint
                    initial_joint_positions_[joint_name] = position;

                    // Log the initial joint positions
                    RCLCPP_INFO(get_node()->get_logger(),
                                "Initial Joint: %s, Initial Position: %f",
                                joint_name.c_str(), position);
                  }
                }
              }
            });

    RCLCPP_INFO(logger, "SWERVEDRIVE CONTROLLER SUCCESSFULLY CONFIGURED ...");

    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

    realtime_odometry_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    std::string tf_prefix = "";
    tf_prefix = std::string(get_node()->get_namespace());

    if (tf_prefix == "/") {
      tf_prefix = "";
    }

    else {
      tf_prefix = tf_prefix + "/";
    }

    const auto odom_frame_id = tf_prefix + odometry_topic_;
    const auto base_frame_id = tf_prefix + base_footprint_;

    auto &odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = odom_frame_id;
    odometry_message.child_frame_id = base_frame_id;

    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

    odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(
        rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr std::size_t NUM_DIMENSIONS = 6;
    for (std::size_t index = 0; index < 6; ++index) {
      // 0, 7, 14, 21, 28, 35
      const std::size_t diagonal_index = NUM_DIMENSIONS * index + index;
      odometry_message.pose.covariance[diagonal_index] =
          pose_covariance_diagonal_[index];
      odometry_message.twist.covariance[diagonal_index] =
          twist_covariance_diagonal_[index];
    }

    odometry_transform_publisher_ =
        get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

    realtime_odometry_transform_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);

    auto &odometry_transform_message =
        realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id =
        odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id =
        base_frame_id;

    previous_update_timestamp_ = get_node()->get_clock()->now();
  }

  catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "EXCEPTION DURING on_configure: %s", e.what());
    return CallbackReturn::ERROR;
  }

  std::chrono::seconds sleep_duration(1);
  rclcpp::sleep_for(sleep_duration);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
SwerveController::update(const rclcpp::Time &time,
                         const rclcpp::Duration &period) {
  // RCLCPP_INFO(get_node()->get_logger(), "[SWERVE_DRIVE_CONTROLLER] INSIDE
  // on_update...");
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted_) {
      halt();
      is_halted_ = true;
    }

    return controller_interface::return_type::OK;
  }

  const auto current_time = time;
  std::shared_ptr<Twist> last_command_msg = std::make_shared<Twist>();
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr) {
    last_command_msg = std::make_shared<Twist>();
    last_command_msg->header.stamp = current_time;
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;

    received_velocity_msg_ptr_.set(
        last_command_msg); // Update the shared pointer

    RCLCPP_WARN(logger, "No velocity command received, using zero velocity");
  }

  const auto age_of_last_command =
      current_time - last_command_msg->header.stamp;

  if (age_of_last_command > cmd_vel_timeout_) {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  Twist command = *last_command_msg;
  double &linear_x_cmd = command.twist.linear.x;
  double &linear_y_cmd = command.twist.linear.y;
  double &angular_cmd = command.twist.angular.z;

  double x_offset = wheel_params_.x_offset;
  double y_offset = wheel_params_.y_offset;
  double radius = wheel_params_.radius;

  // Joint Positions
  std::vector<double> joint_distances(6, 0.0); // 6 joints, initialized to 0

  // TODO: Debug this section later
  try {
    // Iterate over all joints in the joint state message
    if (last_joint_state_msg_ != nullptr &&
        !last_joint_state_msg_->name.empty()) {
      for (size_t i = 0; i < last_joint_state_msg_->name.size(); ++i) {
        if (i >= last_joint_state_msg_->position.size()) {
          RCLCPP_ERROR(logger,
                       "Mismatch in JointState message: name.size() = %zu, "
                       "position.size() = "
                       "%zu",
                       last_joint_state_msg_->name.size(),
                       last_joint_state_msg_->position.size());
          break;
        }
        const std::string &joint_name = last_joint_state_msg_->name[i];
        const double current_position = last_joint_state_msg_->position[i];
        double distance_moved = 0.0;

        auto joint_it =
            std::find(joint_names_.begin(), joint_names_.end(), joint_name);

        if (joint_it != joint_names_.end()) {
          size_t joint_index = joint_it - joint_names_.begin();

          if (std::find(wheel_joints_.begin(), wheel_joints_.end(),
                        joint_name) != wheel_joints_.end()) {
            distance_moved =
                (current_position - initial_joint_positions_[joint_name]) *
                wheel_params_.radius;
          } else {
            distance_moved =
                (current_position - initial_joint_positions_[joint_name]);
          }

          if (joint_index < joint_distances.size()) {
            joint_distances[joint_index] = distance_moved;
          }
        }
      }
    } else {
      RCLCPP_WARN(logger, "Joint state name vector is empty!");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Error in joint state processing: %s", e.what());
  }

  // Compute Wheel Velocities and Positions
  auto states = swerve_kinematics_->ComputeStates(linear_x_cmd, linear_y_cmd,
                                                  angular_cmd, joint_distances);

  // Extract wheel velocities and axle angles from the optimized states
  std::vector<double> wheel_velocities_ = {
      states.front.speed.value(),    // Extract front module velocity
      states.backLeft.speed.value(), // Extract back left module velocity
      states.backRight.speed.value() // Extract back right module velocity
  };

  std::vector<double> axle_angles_ = {
      states.front.angle.Radians().value(),    // Extract front module angle
      states.backLeft.angle.Radians().value(), // Extract back left module angle
      states.backRight.angle.Radians()
          .value() // Extract back right module angle
  };

  try {
    // Set velocities for wheels
    for (size_t i = 0; i < wheel_joints_.size(); ++i) {
      try {
        swerve_hardware_handle_->setSpeed(wheel_joints_[i],
                                          wheel_velocities_[i]);
      } catch (const std::out_of_range &) {
        RCLCPP_INFO(logger, "Wheel Handle not found for joint: %s",
                    wheel_joints_[i].c_str());
      }
    }

    // Set positions for axles
    for (size_t i = 0; i < axle_joints_.size(); ++i) {
      try {
        swerve_hardware_handle_->setPosition(axle_joints_[i], axle_angles_[i]);
      } catch (const std::out_of_range &) {
        RCLCPP_INFO(logger, "Axle Handle not found for joint: %s",
                    axle_joints_[i].c_str());
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Exception caught: %s", e.what());
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  // Update odometry
  frc::Rotation2d rotation{units::radian_t(last_gyro_angle_)};
  std::array<frc::SwerveModulePosition, 3> module_positions = {
      frc::SwerveModulePosition{
          units::meter_t{joint_distances[0]},
          units::radian_t{joint_distances[1]}}, // Front wheel
      frc::SwerveModulePosition{
          units::meter_t{joint_distances[2]},
          units::radian_t{joint_distances[3]}}, // Left rear wheel
      frc::SwerveModulePosition{
          units::meter_t{joint_distances[4]},
          units::radian_t{joint_distances[5]}} // Right rear wheel
  };

  // m_pose = m_odometry.Update(rotation, module_positions);
  m_pose = m_odometry->Update(rotation, module_positions);

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, m_pose.Rotation().Radians().value());

  bool should_publish = false;

  try {
    if (previous_publish_timestamp_ + publish_period_ < time) {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  } catch (const std::runtime_error &) {
    // Handle exceptions when the time source changes and initialize publish
    // timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish) {
    if (realtime_odometry_publisher_ &&
        realtime_odometry_publisher_->trylock()) {
      auto &odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = m_pose.X().value();
      odometry_message.pose.pose.position.y = m_pose.Y().value();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (realtime_odometry_transform_publisher_ &&
        realtime_odometry_transform_publisher_->trylock()) {
      auto &transform =
          realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = m_pose.X().value();
      transform.transform.translation.y = m_pose.Y().value();
      // transform.transform.translation.z = 0.0;     // Add this for
      // completeness
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_activate...");

  swerve_hardware_handle_ = create_swerve_hardware(wheel_joints_, axle_joints_);

  is_halted_ = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(),
               "Subscriber and publisher are now active.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SwerveController::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_deactivate...");

  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_cleanup...");

  if (!reset()) {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_error...");

  if (!reset()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool SwerveController::reset() {
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  auto zero_twist = std::make_shared<Twist>();
  zero_twist->header.stamp = get_node()->get_clock()->now();
  zero_twist->twist.linear.x = 0.0;
  zero_twist->twist.linear.y = 0.0;
  zero_twist->twist.angular.z = 0.0;
  received_velocity_msg_ptr_.set(zero_twist);

  is_halted_ = false;
  return true;
}

CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE on_shutdown...");

  return CallbackReturn::SUCCESS;
}

void SwerveController::halt() {
  RCLCPP_INFO(get_node()->get_logger(),
              "[SWERVE_DRIVE_CONTROLLER] INSIDE halt...");

  for (size_t i = 0; i < wheel_joints_.size(); ++i) {
    try {
      // Set the velocity for each wheel joint
      swerve_hardware_handle_->setSpeed(wheel_joints_[i], 0.0);
    } catch (const std::out_of_range &) {
      // Log a message if the handle for a joint is not found
      RCLCPP_INFO(get_node()->get_logger(),
                  "Wheel Handle not found for joint: %s",
                  wheel_joints_[i].c_str());
    }
  }

  RCLCPP_WARN(get_node()->get_logger(),
              "-----HALT CALLED : STOPPING ALL MOTORS-----");
}

void SwerveHardware::addWheelHandle(const std::string &joint_name,
                                    CommandInterfaceRef command_interface) {
  wheel_handles_.emplace(joint_name, command_interface);
}

void SwerveHardware::addAxleHandle(const std::string &joint_name,
                                   CommandInterfaceRef command_interface) {
  axle_handles_.emplace(joint_name, command_interface);
}

void SwerveHardware::setPosition(const std::string &axle_name,
                                 double position) {
  if (axle_handles_.find(axle_name) != axle_handles_.end()) {
    axle_handles_.at(axle_name).get().set_value(position);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SwerveHardware"),
                "Axle handle not found for: %s", axle_name.c_str());
  }
}

void SwerveHardware::setSpeed(const std::string &wheel_name, double speed) {
  if (wheel_handles_.find(wheel_name) != wheel_handles_.end()) {
    wheel_handles_.at(wheel_name).get().set_value(speed);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SwerveHardware"),
                "Wheel handle not found for: %s", wheel_name.c_str());
  }
}

std::shared_ptr<SwerveHardware> SwerveController::create_swerve_hardware(
    const std::vector<std::string> &wheel_joints,
    const std::vector<std::string> &axle_joints) {
  auto swerve_hardware = std::make_shared<SwerveHardware>();

  for (const auto &joint_name : wheel_joints) {
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto &interface) {
          return interface.get_name() == (joint_name + "/velocity") &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });
    if (command_handle != command_interfaces_.end()) {
      swerve_hardware->addWheelHandle(joint_name, std::ref(*command_handle));
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unable to find command handle for wheel joint: %s",
                   joint_name.c_str());
    }
  }

  for (const auto &joint_name : axle_joints) {
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto &interface) {
          return interface.get_name() == (joint_name + "/position") &&
                 interface.get_interface_name() == HW_IF_POSITION;
        });
    if (command_handle != command_interfaces_.end()) {
      swerve_hardware->addAxleHandle(joint_name, std::ref(*command_handle));
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unable to find command handle for axle joint: %s",
                   joint_name.c_str());
    }
  }
  return swerve_hardware;
}

} // namespace swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(swerve_drive_controller::SwerveController,
                            controller_interface::ControllerInterface)
