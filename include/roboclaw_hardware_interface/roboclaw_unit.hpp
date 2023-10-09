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

#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_

#include <map>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

class RoboClawUnit
{
public:
  std::array<MotorJoint::SharedPtr, 2> joints;

  RoboClawUnit(
    roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1,
    MotorJoint::SharedPtr m2);

  // Read the encoder counts from the roboclaw and update position state
  void read();

  // Conver the velocity command to tick rate request and write to the roboclaw
  void write();

private:
  roboclaw_serial::Interface::SharedPtr interface_;
  const uint8_t address_;

  // RoboClaw serial driver messages
  roboclaw_serial::DriveM1M2WithSignedSpeed tick_rate_command_;
  roboclaw_serial::EncoderCounters encoder_state_;
};
}  // namespace roboclaw_hardware_interface

#endif  // ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT_HPP_
