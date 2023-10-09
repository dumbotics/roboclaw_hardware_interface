// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

#include <iostream>
#include <roboclaw_serial/device.hpp>

namespace roboclaw_hardware_interface
{

CallbackReturn RoboClawHardwareInterface::on_init(const HardwareInfo & hardware_info)
{
  // Validate serial port parameter
  std::string serial_port;
  try {
    serial_port = hardware_info.hardware_parameters.at("serial_port");
  } catch (const std::out_of_range &) {
    std::cerr << "Serial port must be defined as a hardware parameters." << std::endl;
    return CallbackReturn::ERROR;
  }

  // Try to establish a connection to the roboclaw
  try {
    // Read the serial port from hardware parameters
    auto device = std::make_shared<roboclaw_serial::SerialDevice>(serial_port);
    interface_ = std::make_shared<roboclaw_serial::Interface>(device);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return CallbackReturn::FAILURE;
  }

  // Validate parameters describing roboclaw joint configurations
  RoboClawConfiguration config;
  try {
    config = parse_roboclaw_configuration(hardware_info);
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
    return CallbackReturn::ERROR;
  }

  // Initialize each roboclaw unit from validated configuration
  for (auto & [roboclaw_address, joints] : config) {
    roboclaw_units_.push_back(
      RoboClawUnit(interface_, roboclaw_address, joints["M1"], joints["M2"]));
  }

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RoboClawHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto roboclaw : roboclaw_units_) {
    for (auto & joint : roboclaw.joints) {
      if (joint) {
        state_interfaces.emplace_back(joint->name, "position", joint->getPositionStatePtr());
      }
    }
  }
  return state_interfaces;
}

std::vector<CommandInterface> RoboClawHardwareInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto roboclaw : roboclaw_units_) {
    for (auto & joint : roboclaw.joints) {
      if (joint) {
        command_interfaces.emplace_back(joint->name, "velocity", joint->getVelocityCommandPtr());
      }
    }
  }
  return command_interfaces;
}

return_type RoboClawHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & roboclaw : roboclaw_units_) {
    roboclaw.write();
  }
  return return_type::OK;
}

return_type RoboClawHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & roboclaw : roboclaw_units_) {
    roboclaw.read();
  }
  return return_type::OK;
}

RoboClawConfiguration RoboClawHardwareInterface::parse_roboclaw_configuration(
  const HardwareInfo & hardware_info)
{
  // Define the configuration map
  RoboClawConfiguration roboclaw_config;

  // Loop over all motors and associate them with their respective roboclaws
  for (auto joint : hardware_info.joints) {
    // We currently only support velocity command interfaces
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "velocity") {
      throw std::runtime_error(
              "Invalid command interface for " + joint.name +
              ". Only velocity command interfaces are supported.");
    }

    // We currently only support position state interfaces
    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "position") {
      throw std::runtime_error(
              "Invalid state interface for " + joint.name +
              ". Only position state interfaces are supported.");
    }

    // Capture and validate parameters
    uint8_t roboclaw_address;
    try {
      roboclaw_address = static_cast<uint8_t>(stoi(joint.parameters.at("address")));

      if (roboclaw_address < 0x80 || roboclaw_address > 0x87) {
        throw std::runtime_error(joint.name + ": Addresses must be in the range [128:136]");
      }
    } catch (const std::invalid_argument & e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(joint.name + ": Address must be an integer.");
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error("Problem looking up address and converting to uint8_t");
    }

    // Get the tick count per wheel rotation value
    int qppr;
    try {
      qppr = stoi(joint.parameters.at("qppr"));
    } catch (const std::out_of_range &) {
      throw std::runtime_error("qppr is not set for " + joint.name);
    } catch (const std::invalid_argument &) {
      throw std::runtime_error("qppr is not numeric for " + joint.name);
    }

    // Get the type of motor from joint parameters
    std::string motor_type;
    try {
      motor_type = joint.parameters.at("motor_type");
    } catch (const std::out_of_range &) {
      throw std::runtime_error(
              "Motor type not set for " + joint.name + ". It must be either M1 or M2.");
    }

    // Ensure that the motor type is valid
    if (motor_type != "M1" && motor_type != "M2") {
      throw std::runtime_error(
              "Motor type for " + joint.name + " must be either M1 or M2 (" + motor_type +
              " provided).");
    }

    // Ensure that a key exists for this address, otherwise initialize default values
    roboclaw_config.emplace(
      roboclaw_address,
      std::map<std::string, MotorJoint::SharedPtr>({{"M1", nullptr}, {"M2", nullptr}}));

    // Ensure that this motor has not already been configured
    if (!roboclaw_config[roboclaw_address][motor_type]) {
      // Set configuration parameters for this motor
      roboclaw_config[roboclaw_address][motor_type] =
        std::make_shared<MotorJoint>(joint.name, qppr);
    } else {
      throw std::runtime_error(
              "Bad motor type " + motor_type + " specified for joint " + joint.name);
    }
  }

  return roboclaw_config;
}

}  // namespace roboclaw_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware_interface::RoboClawHardwareInterface, hardware_interface::SystemInterface);
