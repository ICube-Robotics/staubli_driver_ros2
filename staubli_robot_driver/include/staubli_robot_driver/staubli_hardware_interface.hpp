// Copyright 2025 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Thibault Poignonec (thibault.poignonec@gmail.fr)

#ifndef STAUBLI_ROBOT_DRIVER__STAUBLI_HARDWARE_INTERFACE_HPP_
#define STAUBLI_ROBOT_DRIVER__STAUBLI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "staubli_robot_driver/robot_driver.hpp"
#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/communication/protocol.hpp"

namespace staubli_robot_driver {

class StaubliHardwareInterface : public hardware_interface::SystemInterface
{
public:
    StaubliHardwareInterface();
    ~StaubliHardwareInterface();

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_error (const rclcpp_lifecycle::State& previous_state) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::return_type
    read(const rclcpp::Time& time, const rclcpp::Duration& period) final;

    hardware_interface::return_type
    write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

    hardware_interface::return_type
    prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) final;

    hardware_interface::return_type
    perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) final;

private:
    /// Update the internal state from the robot driver
    bool update_state(const RobotStateMessage& state_msg);

private:
    /// Robot driver instance
    std::shared_ptr<RobotDriver> robot_driver_;

    /// Latest received robot state message
    RobotStateMessage state_msg_;

    /// Enum defining the current control mode
    CommandType current_control_mode_;

    // Joint state
    size_t num_joints_;  ///< Number of joints
    std::vector<double> hw_joint_positions_;  ///< Current joint positions
    std::vector<double> hw_joint_velocities_;  ///< Current joint velocities
    std::vector<double> hw_joint_efforts_;  ///< Current joint efforts

    // Joint commands
    std::vector<double> hw_joint_position_commands_;  ///< Current joint position commands
    std::vector<double> hw_joint_velocity_commands_;  ///< Current joint velocity commands
    std::vector<double> hw_joint_effort_commands_;    ///< Current joint effort commands

    // GPIO state
    std::vector<double> hw_digital_inputs_;   ///< Current digital inputs
    std::vector<double> hw_analog_inputs_;  ///< Current analog inputs
    std::vector<double> hw_digital_outputs_;   ///< Current digital outputs
    std::vector<double> hw_analog_outputs_;  ///< Current analog outputs

    // GPIO command
    std::vector<double> hw_digital_output_commands_;   ///< Current digital output commands
    std::vector<double> hw_analog_output_commands_;  ///< Current analog output commands
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__STAUBLI_HARDWARE_INTERFACE_HPP_
