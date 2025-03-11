# ROS2 Swerve Drive Controller

A ROS2-compatible swerve drive controller for a 3-wheeled swerve drive robot with WPILib integration.

## Overview

This package provides a controller implementation for controlling a three-wheeled swerve drive robot in ROS2. By integrating with WPILib, it allows for compatibility with FIRST Robotics Competition (FRC) workflows and components while leveraging ROS2's powerful robotic framework.

## Dependencies

### ROS2 Dependencies
- ROS2 (Humble or newer recommended)
- `controller_interface`
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`

### External Dependencies
- WPILib - *Must be installed separately*

## Installation

### 1. Install WPILib
WPILib must be installed separately as it's not part of the standard ROS ecosystem:

### 2. Install ROS2 Package

```bash
# Create a ROS2 workspace (if you don't already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/PavanSandaka/ros2_swerve_drive_controller.git

# Install ROS dependencies (note the --skip-keys to ignore WPILib)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys="wpilib"

# Build the package
colcon build --packages-select ros2_swerve_drive_controller

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Acknowledgments

- ROS2 Control Working Group for the controller interface
- WPILib team for the FRC robotics library
- The FIRST Robotics Competition community
