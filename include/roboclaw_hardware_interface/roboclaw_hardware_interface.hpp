#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP

#include <map>

#include "hardware_interface/system_interface.hpp"
#include "roboclaw_hardware_interface/motor_joint.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace roboclaw_hardware_interface
{

/// The roboclaw configuration parameters
typedef std::map<uint8_t, std::map<std::string, MotorJoint::SharedPtr>> RoboClawConfiguration;

class RoboClawHardwareInterface : public hardware_interface::SystemInterface
{
public:
  //////////////////
  // CONSTRUCTORS //
  //////////////////

  /// Default constructor for the hardware interface
  RoboClawHardwareInterface() = default;

  ////////////////////////////////
  // SYSTEM INTERFACE OVERRIDES //
  ////////////////////////////////

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
     * \param[in] hardware_info structure with data from URDF.
     * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
     * \returns CallbackReturn::ERROR if any error happens or data are missing.
     */
  CallbackReturn on_init(const HardwareInfo & hardware_info) override;

  /// Exports all state interfaces for this hardware interface.
  /**
     * The state interfaces have to be created and transferred according
     * to the hardware info passed in for the configuration.
     *
     * Note the ownership over the state interfaces is transferred to the caller.
     *
     * \return vector of state interfaces
     */
  std::vector<StateInterface> export_state_interfaces() override;

  /// Exports all command interfaces for this hardware interface.
  /**
     * The command interfaces have to be created and transferred according
     * to the hardware info passed in for the configuration.
     *
     * Note the ownership over the state interfaces is transferred to the caller.
     *
     * \return vector of command interfaces
     */
  std::vector<CommandInterface> export_command_interfaces() override;

  /////////////////////////////////
  //  SYSTEM INTERFACE OVERRIDES //
  /////////////////////////////////

  /// Read the current state values from the actuator.
  /**
     * The data readings from the physical hardware has to be updated
     * and reflected accordingly in the exported state interfaces.
     * That is, the data pointed by the interfaces shall be updated.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Write the current command values to the actuator.
  /**
     * The physical hardware shall be updated with the latest value from
     * the exported command interfaces.
     *
     * \param[in] time The time at the start of this control loop iteration
     * \param[in] period The measured time taken by the last control loop iteration
     * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
     */
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Validate and organize hardware parameters from hardware information.
  /**
     * The hardware info is parsed and validated, checking for parameter
     * consistency and validity. Parameters are organized and stored in the
     * roboclaws_ member variable as a
     * std::map<uint8_t, std::map<std::string, std::optional<MotorConfig>>>,
     * which maps a roboclaw address (uint8_t) to a map of optional motor
     * configurations, the keys for which are strings, either "M1" or "M2"
     *
     * \param[in] hardware_info structure with data from URDF.
     * 
     */
  RoboClawConfiguration parse_roboclaw_configuration(const HardwareInfo & hardware_info);
};
}  // namespace roboclaw_hardware_interface

#endif  //ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP
