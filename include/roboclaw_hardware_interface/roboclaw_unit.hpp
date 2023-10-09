#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT__HPP
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT__HPP

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

#endif  // ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_UNIT__HPP
