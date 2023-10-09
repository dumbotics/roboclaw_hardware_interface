# RoboClaw Hardware Interface for ROS2 Control

The RoboClaw Hardware Interface enables seamless integration of RoboClaw motor controllers into ROS2 robotic systems through the `ros2_control` framework.

## Table of Contents
1. [Package Overview](#package-overview)
2. [Setup & Installation](#setup--installation)
3. [Configuration](#configuration)
4. [Running with ROS2](#running-with-ros2)
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
- Download and install the [roboclaw_serial library](git@github.com:dumbotics/roboclaw_serial.git). You can either download this directly to your ROS2 workspace and let colcon build it, or you can manually build and install using CMake.
- Install all ROS2 requirements:
```
rosdep install roboclaw_hardware_interface
```

2. **Build & Source the Workspace**:

```bash
cd ~/ros2_ws
colcon build --packages-select roboclaw_hardware_interface
source ~/ros2_ws/install/setup.bash
```

## Hardware Configuration

Prior to using this interface, the roboclaw device must be configured. Please consult the the BasicMicro user manual. Note that configuration requires installation of the Motion Studio application in a Windows environment.

1. Set communication mode to packet serial
2. calibrate velocity PID for each motor
3. Take note of which motor is designated M1, and which is M2.
4. If not known, determine the encoder tick counts per full joint rotation.
   - Add a mark or piece of tape on the wheel
   - Zero the encoder count in Motion Studio
   - Rotate the wheel and count the number of rotations (more is better)
   - Calculate the average ticks per wheel rotation

### Software 

1. **Update Robot's URDF**:

Your robot's URDF must contain configurations for the RoboClaw Hardware Interface. This includes specifying the serial port, command interfaces, state interfaces, and other essential parameters.

```xml
<ros2_control name="RoboClawSystem" type="system">
  <hardware>
    <plugin>roboclaw_hardware_interface/RoboClawHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
  </hardware>
  <joint name="joint_name">
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

### Essential Parameters:

- **`serial_port`**: The serial port for communication with RoboClaw, e.g., `/dev/ttyACM0`.
- **`address`**: RoboClaw address for the joint; it should be in the range [128:136].
- **`qppr`**: Ticks count per wheel rotation.
- **`motor_type`**: Type of motor; it can be either "M1" or "M2".

2. **Configure Controller**:

Prepare a configuration YAML for the controller manager and controllers. This YAML will detail the type of controllers, their names, and other relevant parameters.

## Running with ROS2

After completing the setup and configuration:

1. Start the Controller Manager with the RoboClaw Hardware Interface.
2. Enable the required controllers.
3. Dispatch commands through ROS2 tools or your application.

## Development & Contribution

- **Testing**: Before making contributions, confirm all tests run without errors:
  ```bash
  colcon test --packages-select roboclaw_hardware_interface
  ```

- **Documentation**: To generate package documentation, make sure Doxygen is installed and run:
  ```bash
  cd ~/ros2_ws/src/roboclaw_hardware_interface
  doxygen Doxyfile
  ```

For Python scripts or packages:
```bash
python setup.py develop
```

## License

The RoboClaw Hardware Interface is provided under an Apache 2.0 license. For full licensing details, consult the `LICENSE` file. Copyright Eric Cox 2023.
