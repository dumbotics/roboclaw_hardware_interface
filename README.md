# RoboClaw Hardware Interface for ROS2 Control

[![ROS 2 CI with Docker](https://github.com/dumbotics/roboclaw_hardware_interface/actions/workflows/ros2-build-test.yml/badge.svg?branch=main)](https://github.com/dumbotics/roboclaw_hardware_interface/actions/workflows/ros2-build-test.yml)

The RoboClaw Hardware Interface enables seamless integration of RoboClaw motor controllers into ROS2 robotic systems through the `ros2_control` framework.

This packages has only been tested with ROS2 humble.

## Table of Contents
1. [Package Overview](#package-overview)
2. [Setup & Installation](#setup--installation)
3. [Configuration](#configuration)
4. [Running with ROS2](#example)
5. [Development & Contribution](#development--contribution)
6. [License](#license)

## Package Overview

This package allows efficient control and monitoring of RoboClaw motor controllers within ROS2 ecosystems by providing a ros2_control system interface to the BasicMicro RoboClaw. The interface is currently limited to velocity control (joint angular velocity request) and position state (joint angle feedback), which is sufficient for integration with the [Differential Driver COntroller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).

## Setup & Installation

### Prerequisites

A working ROS2 environment is required. If you don't have ROS2 installed, please follow the ROS2 installation guide. Ensure that rosdep definitions are current (`rosdep update`).

1. **Clone the Repository**:

```bash
cd ~/ros2_ws/src # Or whatever your ros2 workspace is named
git clone https://github.com/dumbotics/roboclaw_hardware_interface.git
```

2. **Install Dependencies**
- Download and install the [roboclaw_serial library](https://github.com/dumbotics/roboclaw_serial). You can either download this directly to your ROS2 workspace and let colcon build it, or you can manually build and install using CMake.
- Install all ROS2 requirements:
```
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
```

2. **Build & Source the Workspace**:

```bash
colcon build --packages-select roboclaw_hardware_interface
source ~/ros2_ws/install/setup.bash
```

## Configuration

### Hardware Configuration

Prior to using this interface, the roboclaw device must be configured. Please consult the the BasicMicro user manual. Note that configuration requires installation of the Motion Studio application in a Windows environment.

1. Set communication mode to packet serial
2. calibrate velocity PID for each motor
3. Take note of which motor is designated M1, and which is M2.
4. If not known, determine the encoder tick counts per full joint rotation.
   - Add a mark or piece of tape on the wheel
   - Zero the encoder count in Motion Studio
   - Rotate the wheel and count the number of rotations (more is better)
   - Calculate the average ticks per wheel rotation

### Software Configuration 

Hardware and joint parameters must be provide to the controller manager via a URDF configuration file.

1. **Update Robot's URDF**:

The base controller parses hardware and software parameters from URDF and provides them to the hardware interface.

An example .xacro file for a differential system with a left and right wheel is provided below:

```xml
<ros2_control name="RoboClawSystem" type="system">
  <hardware>
    <plugin>roboclaw_hardware_interface/RoboClawHardwareInterface</plugin>
    <param name="serial_port">/dev/roboclaw</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <param name="address">128</param>
    <param name="qppr">500</param>
    <param name="motor_type">M2</param>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <param name="address">128</param>
    <param name="qppr">500</param>
    <param name="motor_type">M1</param>
    <!-- Add other configurations as necessary -->
  </joint>
  <!-- Add other joints as necessary -->
</ros2_control>
```

The hardware interface will validate parameters automatically and fail to start if required parameters are not present or incorrectly-formatted.

### Required Parameters:

- **`serial_port`**: The serial port for communication with the RoboClaw configuration, e.g., `/dev/roboclaw`.
- **`address`**: RoboClaw address for the unit associated with the joint; per the roboclaw user manual, the address be range [128:136].
- **`qppr`**: Ticks count per wheel rotation. This can be either measured as described in the hardware configuration section, or determined by multiply the encoder count by the transmission ratio.
- **`motor_type`**: Type of motor; it can be either "M1" or "M2". This should be determined as described in the hardware configuration section.

2. **Configure Controller**:

Since the hardware_interface will be executed by a controller, a onfiguration YAML for the controller manager and the controllers is required. An example for the `differential_driver_controller` is provided below:

```yml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diffbot_base_controller:
      type: diff_drive_controller/DiffDriveController

diffbot_base_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    # These are the nominal odometry parameters for the differential drive robot
    wheel_separation: 0.35  # The distance between the wheels
    wheel_radius: 0.095  # The radius of each wheel

    # Corrections to nominal odometry parameters
    # (e.g., if the left wheel has a different radius than the right wheel)
    # Consider generating these corrections from a calibration process
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 100.0
    odom_frame_id: odom
    base_frame_id: base_footprint

    open_loop: false  # Calculate odometry (instead of integrating velocity commands)
    enable_odom_tf: true  # Publish odometry transform from odom_frame_id to base_frame_id

    # If the controller doesn't get a velocity request within this time period, stop.
    # This should be larger than the twist publisher's period
    cmd_vel_timeout: 0.1
    #publish_limited_velocity: true
    use_stamped_vel: false
    #velocity_rolling_window_size: 10

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear.x.has_velocity_limits: true
    linear.x.has_acceleration_limits: true
    linear.x.has_jerk_limits: false
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -0.4
    linear.x.max_acceleration: 1.0
    angular.z.has_velocity_limits: true
    angular.z.has_acceleration_limits: true
    angular.z.has_jerk_limits: false
    angular.z.max_velocity: 1.0
    angular.z.max_acceleration: 1.0

```

## Example

A demo bringup package is provided by [dumbot_bringup](https://github.com/dumbotics/dumbot_bringup). This includes example xacro, configuration, and launch files running the roboclaw with the differential drive controller.

## Development & Contribution

- **Testing**: Before making contributions, confirm all tests run without errors:
  ```bash
  colcon test --packages-select roboclaw_hardware_interface
  ```
<!--
- **Documentation**: To generate package documentation, make sure Doxygen is installed and run:
  ```bash
  cd ~/ros2_ws/src/roboclaw_hardware_interface
  doxygen Doxyfile
  ```
-->

## License

The RoboClaw Hardware Interface is provided under an Apache 2.0 license. For full licensing details, consult the `LICENSE` file. Copyright Eric Cox 2023.
