#ifndef ROBOCLAW_CONTROL__ROBOCLAW_BROADCASTER_HPP_
#define ROBOCLAW_CONTROL__ROBOCLAW_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace roboclaw_broadcaster
{

class RoboClawBroadcaster : public controller_interface::ControllerInterface
{
public:

  RoboClawBroadcaster()=default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_init() override;

protected:

  using BatteryStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::BatteryState>;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  std::unique_ptr<BatteryStatePublisher> rt_battery_publisher_;

  using TemperaturePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Temperature>;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  std::unique_ptr<TemperaturePublisher> rt_temperature_publisher_;
};

}  // namespace controller_interface

#endif  // ROBOCLAW_CONTROL__ROBOCLAW_BROADCASTER_HPP_
