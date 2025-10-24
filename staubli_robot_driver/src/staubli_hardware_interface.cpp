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

#include "staubli_robot_driver/staubli_hardware_interface.hpp"

#include <cmath>

#include <limits>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace staubli_robot_driver
{

double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
}

double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

StaubliHardwareInterface::StaubliHardwareInterface()
: robot_driver_(nullptr),
  current_control_mode_(CommandType::STOP),
  num_joints_(0)
{
}

StaubliHardwareInterface::~StaubliHardwareInterface()
{
    if (robot_driver_) {
        robot_driver_->disconnect();
    }
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_init(
const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != \
        hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize number of joints (using params.hardware_info instead of info_)
    num_joints_ = params.hardware_info.joints.size();

    if (num_joints_ == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "No joints defined in robot description");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
    "Initialized with %zu joints", num_joints_);

    // Initialize vectors
    hw_joint_positions_.resize(num_joints_, 0.0);
    hw_joint_velocities_.resize(num_joints_, 0.0);
    hw_joint_efforts_.resize(num_joints_, 0.0);
    hw_joint_position_commands_.resize(num_joints_, 0.0);
    hw_joint_velocity_commands_.resize(num_joints_, 0.0);
    hw_joint_effort_commands_.resize(num_joints_, 0.0);
    std::fill(
        hw_joint_position_commands_.begin(),
        hw_joint_position_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());
    std::fill(
        hw_joint_velocity_commands_.begin(),
        hw_joint_velocity_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());
    std::fill(
        hw_joint_effort_commands_.begin(),
        hw_joint_effort_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());

    // Initialize GPIO vectors (16 digital IOs, 4 analog IOs for Staubli robots)
    hw_digital_inputs_.resize(16, 0.0);
    hw_digital_outputs_.resize(16, 0.0);
    hw_digital_output_commands_.resize(16, 0.0);
    hw_analog_inputs_.resize(4, 0.0);
    hw_analog_outputs_.resize(4, 0.0);
    hw_analog_output_commands_.resize(4, 0.0);

    // Create robot driver
    robot_driver_ = std::make_shared<RobotDriver>();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("StaubliHardwareInterface"), "Configuring hardware interface");

    // Parse HW parameters
    std::string robot_ip = "";
    int control_port = 0;
    int diagnostic_port = 0;
    int local_control_port = 0;
    int local_diagnostic_port = 0;

    size_t num_digital_inputs = 0;
    size_t num_digital_outputs = 0;
    size_t num_analog_inputs = 0;
    size_t num_analog_outputs = 0;
    try {
        robot_ip = info_.hardware_parameters.at("robot_ip");
        control_port = std::stoi(info_.hardware_parameters["control_port"]);
        diagnostic_port = std::stoi(info_.hardware_parameters["diagnostic_port"]);
        local_control_port = std::stoi(info_.hardware_parameters["local_control_port"]);
        local_diagnostic_port = std::stoi(info_.hardware_parameters["local_diagnostic_port"]);

        num_digital_inputs = std::stoul(info_.hardware_parameters["num_digital_inputs"]);
        num_digital_outputs = std::stoul(info_.hardware_parameters["num_digital_outputs"]);
        num_analog_inputs = std::stoul(info_.hardware_parameters["num_analog_inputs"]);
        num_analog_outputs = std::stoul(info_.hardware_parameters["num_analog_outputs"]);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Failed to parse hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }


    RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
        "Connecting to robot at %s", robot_ip.c_str());

    // Configure robot driver connection
    RobotDriver::NetworkConfig net_config;
    net_config.robot_ip = robot_ip;
    net_config.local_control_port = static_cast<uint16_t>(local_control_port);
    net_config.local_diagnostics_port = static_cast<uint16_t>(local_diagnostic_port);
    net_config.control_port = static_cast<uint16_t>(control_port);
    net_config.diagnostics_port = static_cast<uint16_t>(diagnostic_port);

    if (!robot_driver_->connect(net_config, 5000)) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "Failed to connect to robot");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
        "Successfully connected to robot");

    // Resize IO if needed
    if (num_digital_inputs < 16) {
        hw_digital_inputs_.resize(num_digital_inputs, 0.0);
    } else if (num_digital_inputs > 16) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Number of digital inputs exceeds 16!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (num_digital_outputs < 16) {
        hw_digital_outputs_.resize(num_digital_outputs, 0.0);
        hw_digital_output_commands_.resize(num_digital_outputs, 0.0);
    } else if (num_digital_outputs > 16) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Number of digital outputs exceeds 16!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (num_analog_inputs < 4) {
        hw_analog_inputs_.resize(num_analog_inputs, 0.0);
    } else if (num_analog_inputs > 4) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Number of analog inputs exceeds 4!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (num_analog_outputs < 4) {
        hw_analog_outputs_.resize(num_analog_outputs, 0.0);
        hw_analog_output_commands_.resize(num_analog_outputs, 0.0);
    } else if (num_analog_outputs > 4) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Number of analog outputs exceeds 4!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("StaubliHardwareInterface"),
        "Activating hardware interface");

    // Set initial command mode
    current_control_mode_ = CommandType::STOP;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("StaubliHardwareInterface"),
        "Cleaning up hardware interface");

    if (robot_driver_) {
        robot_driver_->disconnect();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("StaubliHardwareInterface"),
        "Shutting down hardware interface");

    if (robot_driver_ && robot_driver_->is_connected()) {
        if (state_msg_.control_state != CommandType::STOP) {
            robot_driver_->send_stop_all_command();
        }
        robot_driver_->disconnect();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StaubliHardwareInterface::on_error (
  const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_DEBUG(rclcpp::get_logger("StaubliHardwareInterface"),
        "Handling error state in hardware interface");

    if (robot_driver_ && robot_driver_->is_connected()) {
        robot_driver_->send_stop_all_command();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "Robot driver not connected during error handling");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StaubliHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export joint state interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto & joint_info = info_.joints[i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_info.name, hardware_interface::HW_IF_POSITION, &hw_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_info.name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_info.name, hardware_interface::HW_IF_EFFORT, &hw_joint_efforts_[i]));
  }

  // Export GPIO state interfaces
  for (size_t i = 0; i < hw_digital_inputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "digital_input_" + std::to_string(i), "digital_input", &hw_digital_inputs_[i]));
  }

  for (size_t i = 0; i < hw_analog_inputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "analog_input_" + std::to_string(i), "analog_input", &hw_analog_inputs_[i]));
  }

  for (size_t i = 0; i < hw_digital_outputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "digital_output_" + std::to_string(i), "digital_output", &hw_digital_outputs_[i]));
  }

  for (size_t i = 0; i < hw_analog_outputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      "analog_output_" + std::to_string(i), "analog_output", &hw_analog_outputs_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
StaubliHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export joint command interfaces (position, velocity, effort)
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto & joint_info = info_.joints[i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_POSITION, &hw_joint_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocity_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_EFFORT, &hw_joint_effort_commands_[i]));
  }

  // Export GPIO command interfaces
  for (size_t i = 0; i < hw_digital_output_commands_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "digital_output_" + std::to_string(i), "digital_output", &hw_digital_output_commands_[i]));
  }

  for (size_t i = 0; i < hw_analog_output_commands_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "analog_output_" + std::to_string(i), "analog_output", &hw_analog_output_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type StaubliHardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Read from robot driver
    if (!robot_driver_ || !robot_driver_->is_connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"), "Robot driver not connected");
        return hardware_interface::return_type::ERROR;
    }

    if (!robot_driver_->read_robot_state(state_msg_)) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"), "Failed to read robot state");
        return hardware_interface::return_type::ERROR;
    }

    // Update internal state
    if (!update_state(state_msg_)) {
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StaubliHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    // Check connection with robot
    if (!robot_driver_ || !robot_driver_->is_connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"), "Robot driver not connected");
        return hardware_interface::return_type::ERROR;
    }

    // Check for error state
    if (current_control_mode_ != CommandType::STOP) {
        if (state_msg_.error_state || state_msg_.control_state == CommandType::INVALID) {
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Robot is in error state!");
            robot_driver_->send_stop_all_command();
            return hardware_interface::return_type::ERROR;
        }
    }

    // Create command message
    RobotCommandMessage cmd_msg;
    cmd_msg.header.sequence_number = static_cast<uint16_t>(
        robot_driver_->get_current_message_sequence());
    cmd_msg.controller_period = 1e3 * period.seconds();  // in milliseconds
    cmd_msg.command_type = current_control_mode_;

    if (cmd_msg.command_type == CommandType::STOP) {
        // Stop command
        std::fill(cmd_msg.command_reference.begin(), cmd_msg.command_reference.end(), 0.0);
    } else if (cmd_msg.command_type == CommandType::JOINT_POSITION) {
        // Position commands
        for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
            // Staubli uses degrees for joint position/velocity
            cmd_msg.command_reference[i] = rad2deg(hw_joint_position_commands_[i]);
        }
    } else if (cmd_msg.command_type == CommandType::JOINT_VELOCITY) {
        // Velocity commands
        for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
            cmd_msg.command_reference[i] = rad2deg(hw_joint_velocity_commands_[i]);
        }
    } else if (cmd_msg.command_type == CommandType::JOINT_TORQUE) {
        // Effort commands
        for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
            cmd_msg.command_reference[i] = hw_joint_effort_commands_[i];
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "Unsupported control mode: %d", static_cast<int>(cmd_msg.command_type));
        return hardware_interface::return_type::ERROR;
    }

    // Set digital outputs
    for (size_t i = 0; i < hw_digital_output_commands_.size() && i < 16; ++i) {
        cmd_msg.digital_outputs[i] = (hw_digital_output_commands_[i] > 0.5);
    }

    // Set analog outputs
    for (size_t i = 0; i < hw_analog_output_commands_.size() && i < 4; ++i) {
        cmd_msg.analog_outputs[i] = hw_analog_output_commands_[i];
    }


    // Check for NaN joint commands
    bool found_nan = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (std::isnan(cmd_msg.command_reference[i])) {
            found_nan = true;
            break;
        }
    }
    if (found_nan) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "NaN detected in command reference");
        robot_driver_->send_stop_all_command();
        return hardware_interface::return_type::ERROR;
    }

    // Send command
    if (!robot_driver_->send_robot_command(cmd_msg)) {
        RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
            "Failed to send robot command");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StaubliHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Determine the new control mode based on the interfaces
    CommandType new_mode = CommandType::STOP;

    for (const auto& interface : start_interfaces) {
    if (interface.find("position") != std::string::npos) {
        new_mode = CommandType::JOINT_POSITION;
        break;
    } else if (interface.find("velocity") != std::string::npos) {
        new_mode = CommandType::JOINT_VELOCITY;
        break;
    } else if (interface.find("effort") != std::string::npos) {
        new_mode = CommandType::JOINT_TORQUE;
        break;
    }
    }

    RCLCPP_INFO(
    rclcpp::get_logger("StaubliHardwareInterface"),
        "Preparing mode switch to: %d", static_cast<int>(new_mode));

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StaubliHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Determine and set the new control mode
    CommandType new_mode = CommandType::STOP;

    for (const auto& interface : start_interfaces) {
        if (interface.find("position") != std::string::npos) {
            new_mode = CommandType::JOINT_POSITION;
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Switched to JOINT_POSITION mode");
            break;
        } else if (interface.find("velocity") != std::string::npos) {
            new_mode = CommandType::JOINT_VELOCITY;
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Switched to JOINT_VELOCITY mode");
            break;
        } else if (interface.find("effort") != std::string::npos) {
            new_mode = CommandType::JOINT_TORQUE;
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Switched to JOINT_TORQUE mode");
            break;
        }
    }

    // Initialize commands
    hw_joint_position_commands_ = hw_joint_positions_;
    std::fill(hw_joint_velocity_commands_.begin(), hw_joint_velocity_commands_.end(), 0.0);
    std::fill(hw_joint_effort_commands_.begin(), hw_joint_effort_commands_.end(), 0.0);


    current_control_mode_ = new_mode;

    RCLCPP_INFO(
    rclcpp::get_logger("StaubliHardwareInterface"),
        "Switched to control mode: %d", static_cast<int>(current_control_mode_));

    return hardware_interface::return_type::OK;
}

bool StaubliHardwareInterface::update_state(const RobotStateMessage& state_msg)
{
    // Update joint positions
    if (true)  // { state_msg.has_joint_positions)
    {
        for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
            hw_joint_positions_[i] = deg2rad(state_msg.joint_positions[i]);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("StaubliHardwareInterface"),
            "State message missing joint positions");
        return false;
    }

    // Update joint velocities
    if (true)  // { state_msg.has_joint_velocities) {
    {
        for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
            hw_joint_velocities_[i] = deg2rad(state_msg.joint_velocities[i]);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("StaubliHardwareInterface"),
            "State message missing joint velocities");
        return false;
    }

    // Update joint efforts
    if (state_msg.has_joint_torques) {
    for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
        hw_joint_efforts_[i] = state_msg.joint_torques[i];
    }
    } else {
        // Efforts may not be available; set to zero
        std::fill(hw_joint_efforts_.begin(), hw_joint_efforts_.end(), 0.0);
    }

    // Update digital inputs
    if (state_msg.has_digital_inputs) {
        for (size_t i = 0; i < hw_digital_inputs_.size() && i < 16; ++i) {
            hw_digital_inputs_[i] = state_msg.digital_inputs[i] ? 1.0 : 0.0;
        }
    }

    // Update analog inputs
    if (state_msg.has_analog_inputs) {
        for (size_t i = 0; i < hw_analog_inputs_.size() && i < 4; ++i) {
            hw_analog_inputs_[i] = state_msg.analog_inputs[i];
        }
    }

    return true;
}

}  // namespace staubli_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  staubli_robot_driver::StaubliHardwareInterface,
  hardware_interface::SystemInterface)
