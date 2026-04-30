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

// C++
#include <memory>
#include <string>
#include <vector>

// ROS2
#include "rclcpp_lifecycle/state.hpp"

// ROS2 Control
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"

// Staubli Robot Driver
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
    /// Map start_interfaces to a CommandType. Returns INVALID for unsupported modes.
    CommandType resolve_mode(const std::vector<std::string>& start_interfaces) const;

    /// Update the internal state from the robot driver
    bool update_state(const RobotStateMessage& state_msg);

private:
    /// Robot driver instance
    std::shared_ptr<RobotDriver> robot_driver_;

    /// Latest received robot state message
    RobotStateMessage state_msg_;

    /// Enum defining the current control mode
    CommandType current_control_mode_;

    /// Pending mode resolved during prepare_command_mode_switch and consumed by perform.
    /// Mirrors the UR driver pattern: prepare caches, perform consumes.
    CommandType pending_mode_;

    /// True when a valid switch has been prepared and is awaiting perform.
    bool switch_prepared_;

    struct SupervisionGpio {
        double operation_mode;
        double operation_mode_status;
        double safety_status;
        double control_sequence_delay;
        // Bool values
        double brakes_released;
        double motion_possible;
        double in_motion;
        double in_error;
        double estop_pressed;
        double wait_for_ack;
    };

    // Copy of supervision GPIO for state interface
    SupervisionGpio supervision_gpio_copy_;

    // Joint state
    size_t num_joints_;  ///< Number of joints
    std::vector<double> hw_joint_positions_;  ///< Current joint positions
    std::vector<double> hw_joint_velocities_;  ///< Current joint velocities
    std::vector<double> hw_joint_efforts_;  ///< Current joint efforts

    // Joint commands
    std::vector<double> hw_joint_position_commands_;  ///< Current joint position commands
    std::vector<double> hw_joint_velocity_commands_;  ///< Current joint velocity commands

    /**
     * @brief Current joint acceleration commands
     *
     * @warning This is used to pass acceleration limits to the joint limiter, not to directly
     * control the robot. Although the command interface is exported, switching to an acceleration
     * control mode is not supported and **a switch to acceleration mode will ALWAYS BE REJECTED**.
     */
    std::vector<double> hw_joint_acceleration_commands_;

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
