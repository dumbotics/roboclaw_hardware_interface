#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

namespace roboclaw_hardware_interface
{

CallbackReturn RoboClawHardwareInterface::on_configure(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_cleanup(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_shutdown(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_activate(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_deactivate(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_error(const State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn RoboClawHardwareInterface::on_init(const HardwareInfo & hardware_info)
{
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RoboClawHardwareInterface::export_state_interfaces()
{
  StateInterface interface("left");
  std::vector<StateInterface> interfaces;
  interfaces.push_back(interface);
  return interfaces;
}

std::vector<CommandInterface> RoboClawHardwareInterface::export_command_interfaces()
{
  return std::vector<CommandInterface>();
}

return_type RoboClawHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return return_type::OK;
}

return_type RoboClawHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return return_type::OK;
}

}  // namespace roboclaw_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware_interface::RoboClawHardwareInterface, hardware_interface::SystemInterface);
