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

const size_t MAX_DIGITAL_IO = 16;
const size_t MAX_ANALOG_IO = 4;

StaubliHardwareInterface::StaubliHardwareInterface()
: robot_driver_(nullptr),
  current_control_mode_(CommandType::STOP),
  pending_mode_(CommandType::STOP),
  switch_prepared_(false),
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
    hw_joint_acceleration_commands_.resize(num_joints_, 0.0);
    std::fill(
        hw_joint_position_commands_.begin(),
        hw_joint_position_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());
    std::fill(
        hw_joint_velocity_commands_.begin(),
        hw_joint_velocity_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());
    std::fill(
        hw_joint_acceleration_commands_.begin(),
        hw_joint_acceleration_commands_.end(),
        std::numeric_limits<double>::quiet_NaN());

    // Initialize GPIO vectors (max : 16 x 2 digital IOs, 4 analog IOs for Staubli robots)
    size_t num_digital_inputs = 0;
    size_t num_digital_outputs = 0;
    size_t num_analog_inputs = 0;
    size_t num_analog_outputs = 0;
    try {
        num_digital_inputs = std::stoul(info_.hardware_parameters["num_digital_inputs"]);
        num_digital_outputs = std::stoul(info_.hardware_parameters["num_digital_outputs"]);
        num_analog_inputs = std::stoul(info_.hardware_parameters["num_analog_inputs"]);
        num_analog_outputs = std::stoul(info_.hardware_parameters["num_analog_outputs"]);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Failed to parse hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Resize IO if needed
    RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
        "Configuring with %zu digital inputs, %zu digital outputs, "
        "%zu analog inputs, %zu analog outputs",
        num_digital_inputs, num_digital_outputs, num_analog_inputs, num_analog_outputs);

    if (num_digital_inputs > MAX_DIGITAL_IO ||
        num_digital_outputs > MAX_DIGITAL_IO ||
        num_analog_inputs > MAX_ANALOG_IO ||
        num_analog_outputs > MAX_ANALOG_IO) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Number of IO exceeds maximum allowed!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_digital_inputs_.resize(num_digital_inputs, 0.0);
    hw_digital_outputs_.resize(num_digital_outputs, 0.0);
    hw_digital_output_commands_.resize(num_digital_outputs, 0.0);
    hw_analog_inputs_.resize(num_analog_inputs, 0.0);
    hw_analog_outputs_.resize(num_analog_outputs, 0.0);
    hw_analog_output_commands_.resize(num_analog_outputs, 0.0);

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
    try {
        robot_ip = info_.hardware_parameters.at("robot_ip");
        control_port = std::stoi(info_.hardware_parameters["control_port"]);
        diagnostic_port = std::stoi(info_.hardware_parameters["diagnostic_port"]);
        local_control_port = std::stoi(info_.hardware_parameters["local_control_port"]);
        local_diagnostic_port = std::stoi(info_.hardware_parameters["local_diagnostic_port"]);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("StaubliHardwareInterface"),
            "Failed to parse hardware parameters: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Connect to robot
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

  std::string robot_prefix = "";
  try {
    robot_prefix = info_.hardware_parameters.at("robot_prefix");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
        "Failed to get robot_prefix parameter: %s", e.what());
  }

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

  // Export supervision GPIO state interfaces
  const std::string gpio_supervision_prefix = robot_prefix + "supervision";

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "operation_mode",
        &supervision_gpio_copy_.operation_mode));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "operation_mode_status",
        &supervision_gpio_copy_.operation_mode_status));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "safety_status",
        &supervision_gpio_copy_.safety_status));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "control_sequence_delay",
        &supervision_gpio_copy_.control_sequence_delay));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "brakes_released",
        &supervision_gpio_copy_.brakes_released));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "motion_possible",
        &supervision_gpio_copy_.motion_possible));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "in_motion",
        &supervision_gpio_copy_.in_motion));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "in_error",
        &supervision_gpio_copy_.in_error));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "estop_pressed",
        &supervision_gpio_copy_.estop_pressed));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_supervision_prefix, "wait_for_ack",
        &supervision_gpio_copy_.wait_for_ack));

  // Export GPIO state interfaces
  for (size_t i = 0; i < hw_digital_inputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      robot_prefix + "gpio",
      "digital_input_" + std::to_string(i), &hw_digital_inputs_[i]
    ));
  }

  for (size_t i = 0; i < hw_analog_inputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      robot_prefix + "gpio",
      "analog_input_" + std::to_string(i), &hw_analog_inputs_[i]
    ));
  }

  /*
  for (size_t i = 0; i < hw_digital_outputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      robot_prefix + "gpio",
      "digital_output_" + std::to_string(i), &hw_digital_outputs_[i]
    ));
  }

  for (size_t i = 0; i < hw_analog_outputs_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      robot_prefix + "gpio",
      "analog_output_" + std::to_string(i), &hw_analog_outputs_[i]
    ));
  }
  */
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
StaubliHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  std::string robot_prefix = "";
  try {
    robot_prefix = info_.hardware_parameters.at("robot_prefix");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
        "Failed to get robot_prefix parameter: %s", e.what());
  }

  // Export joint command interfaces (position & velocity)
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto & joint_info = info_.joints[i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_POSITION, &hw_joint_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocity_commands_[i]));

    // TODO(anyone): remove once acc. limits from r2c config are supported
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_info.name, hardware_interface::HW_IF_ACCELERATION,
        &hw_joint_acceleration_commands_[i]));
  }

  // Export GPIO command interfaces
  for (size_t i = 0; i < hw_digital_output_commands_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      robot_prefix + "gpio",
      "digital_output_" + std::to_string(i),
      &hw_digital_output_commands_[i]));
  }

  for (size_t i = 0; i < hw_analog_output_commands_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      robot_prefix + "gpio",
      "analog_output_" + std::to_string(i),
      &hw_analog_output_commands_[i]));
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

CommandType StaubliHardwareInterface::resolve_mode(
    const std::vector<std::string>& start_interfaces) const
{
    for (const auto& interface : start_interfaces) {
        if (interface.find("position") != std::string::npos) {
            return CommandType::JOINT_POSITION;
        } else if (interface.find("velocity") != std::string::npos) {
            return CommandType::JOINT_VELOCITY;
        } else if (interface.find("effort") != std::string::npos) {
            return CommandType::JOINT_TORQUE;
        } else if (interface.find("acceleration") != std::string::npos) {
            return CommandType::INVALID;
        }
    }
    return CommandType::STOP;
}

hardware_interface::return_type StaubliHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    switch_prepared_ = false;

    // Determine incoming mode (from interfaces being started)
    const CommandType incoming_mode = resolve_mode(start_interfaces);

    // TODO(anyone): consider validating that all interfaces are stopped/started together
    // (e.g., mixed position and velocity interfaces should not be allowed)

    // Validate mode. Acceleration interfaces are exported for joint-limiter support only;
    // ignore them when resolving the effective incoming command mode.
    // INVALID is returned by resolve_mode() only when acceleration is the ONLY match and
    // there are no position / velocity / effort interfaces in start_interfaces.
    const bool only_stop_or_accel_starting =
        (incoming_mode == CommandType::STOP || incoming_mode == CommandType::INVALID);

    // TODO(anyone): once more modes are supported, the transition logic will have to be
    // validated against the current active mode...

    // Determine outgoing mode (from interfaces being stopped)
    // const CommandType outgoing_mode = resolve_mode(stop_interfaces);
    // (void)outgoing_mode;

    switch (incoming_mode) {
        case CommandType::STOP:
            RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
                "prepare_command_mode_switch: incoming STOP command mode");
            break;  // accepted
        case CommandType::JOINT_POSITION:
            RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
                "prepare_command_mode_switch: incoming JOINT_POSITION command mode");
            break;  // accepted
        case CommandType::JOINT_VELOCITY:
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Velocity control is not yet supported, switch rejected");
            return hardware_interface::return_type::ERROR;
        case CommandType::JOINT_TORQUE:
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Torque/effort control is not yet supported, switch rejected");
            return hardware_interface::return_type::ERROR;
        case CommandType::INVALID:
            // Acceleration-only start interfaces: treated as no active joint command mode
            if (!only_stop_or_accel_starting) {
                RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                    "Requested interface is not a supported command mode, switch rejected");
                return hardware_interface::return_type::ERROR;
            }
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Requested interface is not a supported command mode, switch rejected");
            return hardware_interface::return_type::ERROR;
    }

    // Cache resolved mode for perform_command_mode_switch
    pending_mode_ = (incoming_mode == CommandType::INVALID) ? CommandType::STOP : incoming_mode;
    switch_prepared_ = true;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type StaubliHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
    // Consume the mode that was validated and cached in prepare_command_mode_switch.
    // If prepare was never called (e.g. on startup), treat as a no-op STOP switch.
    const CommandType new_mode = switch_prepared_ ? pending_mode_ : CommandType::STOP;
    switch_prepared_ = false;  // consume the prepared mode

    // Validate robot state for JOINT_POSITION before committing the switch.
    if (new_mode == CommandType::JOINT_POSITION) {
        if (state_msg_.error_state) {
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Cannot switch to JOINT_POSITION: robot is in error state");
            current_control_mode_ = CommandType::STOP;
            return hardware_interface::return_type::ERROR;
        }
        if (!state_msg_.motion_possible) {
            RCLCPP_ERROR(rclcpp::get_logger("StaubliHardwareInterface"),
                "Cannot switch to JOINT_POSITION: robot brakes engaged");
            current_control_mode_ = CommandType::STOP;
            return hardware_interface::return_type::ERROR;
        }
        if (state_msg_.operation_mode_status != 0) {
            RCLCPP_WARN(rclcpp::get_logger("StaubliHardwareInterface"),
                "Robot is on HOLD — ensure robot is enabled before commanding motion");
        }
    }

    // Safe-initialize position commands to current position; reset velocity/acceleration.
    // Use std::copy / std::fill (no reallocation) to keep raw pointers in CommandInterfaces valid.
    std::copy(hw_joint_positions_.begin(), hw_joint_positions_.end(),
                hw_joint_position_commands_.begin());
    std::fill(hw_joint_velocity_commands_.begin(), hw_joint_velocity_commands_.end(), 0.0);
    std::fill(hw_joint_acceleration_commands_.begin(), hw_joint_acceleration_commands_.end(), 0.0);

    // Save new mode and log the switch
    RCLCPP_INFO(rclcpp::get_logger("StaubliHardwareInterface"),
        "Switched from control mode: %d to control mode: %d",
        static_cast<int>(current_control_mode_),
        static_cast<int>(new_mode));
    current_control_mode_ = new_mode;

    return hardware_interface::return_type::OK;
}

bool StaubliHardwareInterface::update_state(const RobotStateMessage& state_msg)
{
    // Update joint positions
    if (state_msg.has_joint_positions)
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
    if (state_msg.has_joint_velocities) {
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

    // Update supervision GPIOs
    supervision_gpio_copy_.operation_mode = static_cast<double>(state_msg.operation_mode);
    supervision_gpio_copy_.operation_mode_status = \
        static_cast<double>(state_msg.operation_mode_status);
    supervision_gpio_copy_.safety_status = static_cast<double>(state_msg.safety_status);
    supervision_gpio_copy_.control_sequence_delay = static_cast<double>(state_msg.sequence_delay);
    supervision_gpio_copy_.brakes_released = static_cast<double>(state_msg.brakes_released);
    supervision_gpio_copy_.motion_possible = static_cast<double>(state_msg.motion_possible);
    supervision_gpio_copy_.in_motion = static_cast<double>(state_msg.in_motion);
    supervision_gpio_copy_.in_error = static_cast<double>(state_msg.error_state);
    supervision_gpio_copy_.estop_pressed = static_cast<double>(state_msg.estop_pressed);
    return true;
}

}  // namespace staubli_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  staubli_robot_driver::StaubliHardwareInterface,
  hardware_interface::SystemInterface)
