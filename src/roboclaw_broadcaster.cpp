#include "roboclaw_hardware_interface/roboclaw_broadcaster.hpp"

#include "controller_interface/helpers.hpp"

namespace roboclaw_broadcaster
{

controller_interface::InterfaceConfiguration RoboClawBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration RoboClawBroadcaster::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      "roboclaw/battery_voltage",
      "roboclaw/battery_percent",
      "roboclaw/temperature"
    }
  };
}

controller_interface::return_type RoboClawBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  rt_battery_publisher_->lock();
  rt_battery_publisher_->msg_.header.stamp = time;
  rt_battery_publisher_->msg_.voltage = state_interfaces_[0].get_value();
  rt_battery_publisher_->msg_.percentage = state_interfaces_[1].get_value();
  rt_battery_publisher_->unlockAndPublish();

  rt_temperature_publisher_->lock();
  rt_temperature_publisher_->msg_.header.stamp = time;
  rt_temperature_publisher_->msg_.temperature = state_interfaces_[2].get_value();
  rt_temperature_publisher_->unlockAndPublish();

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RoboClawBroadcaster::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // Configure realtime battery state publisher
  battery_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::BatteryState>("~/battery", rclcpp::SystemDefaultsQoS());
  rt_battery_publisher_ = std::make_unique<BatteryStatePublisher>(battery_state_publisher_);

  rt_battery_publisher_->lock();
  rt_battery_publisher_->msg_.header.frame_id = "roboclaw";
  rt_battery_publisher_->msg_.temperature = std::numeric_limits<float>::quiet_NaN();
  rt_battery_publisher_->msg_.current = std::numeric_limits<float>::quiet_NaN();
  rt_battery_publisher_->msg_.charge = std::numeric_limits<float>::quiet_NaN();
  rt_battery_publisher_->msg_.capacity = std::numeric_limits<float>::quiet_NaN();
  rt_battery_publisher_->msg_.design_capacity = std::numeric_limits<float>::quiet_NaN();
  rt_battery_publisher_->unlock();

  // Configure realtime temperature publisher
  temperature_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", rclcpp::SystemDefaultsQoS());
  rt_temperature_publisher_ = std::make_unique<TemperaturePublisher>(temperature_publisher_);

  rt_temperature_publisher_->lock();
  rt_temperature_publisher_->msg_.header.frame_id = "roboclaw";
  rt_temperature_publisher_->unlock();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboClawBroadcaster::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboClawBroadcaster::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoboClawBroadcaster::on_init()
{
  return CallbackReturn::SUCCESS;
}

}  // namespace roboclaw_broadcaster

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  roboclaw_broadcaster::RoboClawBroadcaster, controller_interface::ControllerInterface);
