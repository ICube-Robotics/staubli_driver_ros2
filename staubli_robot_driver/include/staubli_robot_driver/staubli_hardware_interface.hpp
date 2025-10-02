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

#include "rclcpp_lifecycle/state.hpp"

namespace staubli_robot_driver {

class StaubliHardwareInterface : public hardware_interface::SystemInterface
{
public:
    StaubliHardwareInterface();
    ~StaubliHardwareInterface();

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

    hardware_interface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& previous_state) final;

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
    // Robot driver
    std::shared_ptr<RobotDriver> robot_driver_;

    // Joint state and command vectors
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_;
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__STAUBLI_HARDWARE_INTERFACE_HPP_
