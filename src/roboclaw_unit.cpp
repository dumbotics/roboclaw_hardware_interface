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

#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

#include <map>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

RoboClawUnit::RoboClawUnit(
  roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1,
  MotorJoint::SharedPtr m2)
: address_(address)
{
  // Copy the pointer to the roboclaw interface
  interface_ = interface;

  // Set motor joint configurations
  joints[0] = m1;
  joints[1] = m2;

  // Read min/max main voltage cutoffs directly from roboclaw unit
  interface_->read(main_voltage_cutoff_request_, address_);
  const auto & [min_cutoff, max_cutoff] = main_voltage_cutoff_request_.fields;

  // Cutoffs are provided in 0.1V increments
  main_voltage_cutoff_min_ = 0.1 * static_cast<double>(min_cutoff);
  main_voltage_cutoff_max_ = 0.1 * static_cast<double>(max_cutoff);

  std::cout << "Min Cutoff: " << main_voltage_cutoff_min_ << std::endl;
  std::cout << "Max Cutoff: " << main_voltage_cutoff_max_ << std::endl;
}

// Read the encoder counts from the roboclaw and update position state
void RoboClawUnit::read()
{
  // Read and update position
  interface_->read(encoder_request_, address_);

  // Get constant references to fields in the encoder counts message
  const auto & [m1_ticks, m2_ticks] = encoder_request_.fields;

  interface_->read(current_request_, address_);

  // Get constant references to motor currents
  const auto & [m1_current, m2_current] = current_request_.fields;

  // Convert tick counts to position states for each field if the corresponding joint exists
  if (joints[0]) {
    joints[0]->setPositionState(m1_ticks);
    joints[0]->setEffortState(m1_current);
  }
  if (joints[1]) {
    joints[1]->setPositionState(m2_ticks);
    joints[1]->setEffortState(m2_current);
  }

  // Read and process main voltage
  interface_->read(main_voltage_request_, address_);
  const auto & [voltage] = main_voltage_request_.fields;
  main_voltage_state_ = static_cast<double>(voltage) / 10.0;

  // Process battery percent
  main_voltage_percent_ = (main_voltage_state_ - main_voltage_cutoff_min_) / (main_voltage_cutoff_max_ - main_voltage_cutoff_min_);

  // Read and process temperature
  interface_->read(temperature_request_, address_);
  const auto & [temp] = temperature_request_.fields;
  temperature_state_ = static_cast<double>(temp) / 10.0;
}

// Write the tick rate request to the roboclaw and update
void RoboClawUnit::write()
{
  // Get references to fields in the tick rate command message
  auto & [m1_speed, m2_speed] = tick_rate_command_.fields;

  // Set values to each field if the corresponding joint exists
  if (joints[0]) {
    m1_speed = joints[0]->getTickRateCommand();
  }
  if (joints[1]) {
    m2_speed = joints[1]->getTickRateCommand();
  }

  // Write the rate request to the roboclaw driver
  interface_->write(tick_rate_command_, address_);
}
}  // namespace roboclaw_hardware_interface
