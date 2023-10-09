#pragma once

#include <math.h>

#include <array>
#include <memory>
#include <string>

namespace roboclaw_hardware_interface
{
class MotorJoint
{
private:
  // The radians of rotation per tick
  const double ticks_per_radian_;

  // Desired velocity of the wheel in radians per second
  double velocity_command_ = 0;

  // Position of the wheel in radians
  double position_state_ = 0;

  // Store the prior encoder count for updating position state
  int32_t prior_encoder_count_;
  bool initialized_encoder_count_ = false;

public:
  typedef std::shared_ptr<MotorJoint> SharedPtr;

  // Name of the parent joint
  const std::string name;

  // Constructor
  MotorJoint(const std::string joint_name, const int32_t qppr);

  // Return the tick rate required to execute the current velocity command
  int32_t getTickRateCommand() const;

  // Set the position given the current wheel encoder count
  void setPositionState(const int32_t & encoder_count);

  // Accessor methods to enable ros2_control to access joint interface pointers
  inline double * getPositionStatePtr() { return &position_state_; };
  inline double * getVelocityCommandPtr() { return &velocity_command_; };
};
}  // namespace roboclaw_hardware_interface
