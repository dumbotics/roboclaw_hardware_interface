#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

MotorJoint::MotorJoint(const std::string joint_name, const int32_t qppr)
: ticks_per_radian_(static_cast<double>(qppr) * 0.5 * M_1_PI), name(joint_name)
{
}

int32_t MotorJoint::getTickRateCommand() const
{
  return static_cast<int32_t>(velocity_command_ * ticks_per_radian_);
}

// Set the position given the current wheel encoder count
void MotorJoint::setPositionState(const int32_t & encoder_count)
{
  if (initialized_encoder_count_) {
    // Update the joint angle
    position_state_ +=
      static_cast<double>(encoder_count - prior_encoder_count_) / ticks_per_radian_;
  } else {
    initialized_encoder_count_ = true;
  }
  // Store the prior encoder count for next time
  prior_encoder_count_ = encoder_count;
}
}  // namespace roboclaw_hardware_interface
