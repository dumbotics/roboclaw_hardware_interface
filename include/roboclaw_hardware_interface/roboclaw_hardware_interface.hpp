#ifndef ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP
#define ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP

#include "hardware_interface/system_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using rclcpp_lifecycle::State;

namespace roboclaw_hardware_interface
{
class RoboClawHardwareInterface : public hardware_interface::SystemInterface
{
public:

    //////////////////
    // CONSTRUCTORS //
    //////////////////

    /// Default constructor for the hardware interface
    RoboClawHardwareInterface()=default;

    //////////////////////////////
    // LIFECYCLE NODE OVERRIDES //
    //////////////////////////////

    /// Callback function for configure transition
    /**
     * \return true by default
     */
    CallbackReturn on_configure(const State & previous_state) override;

    /// Callback function for cleanup transition
    /**
     * \return true by default
     */
    CallbackReturn on_cleanup(const State & previous_state) override;

    /// Callback function for shutdown transition
    /**
     * \return true by default
     */
    CallbackReturn on_shutdown(const State & previous_state) override;

    /// Callback function for activate transition
    /**
     * \return true by default
     */
    CallbackReturn on_activate(const State & previous_state) override;

    /// Callback function for deactivate transition
    /**
     * \returns true by default
     */
    CallbackReturn on_deactivate(const State & previous_state) override;

    /// Callback function for erroneous transition
    /**
     * \returns false by default
     */
    CallbackReturn on_error(const State & previous_state) override;

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
};
}

#endif //ROBOCLAW_HARDWARE_INTERFACE__ROBOCLAW_HARDWARE_INTERFACE__HPP
