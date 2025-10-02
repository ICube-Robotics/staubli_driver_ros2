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

#include "staubli_robot_driver/robot_driver.hpp"
#include "staubli_robot_driver/robot_driver_helpers.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

namespace staubli_robot_driver {

RobotDriver::RobotDriver()
: logger_(rclcpp::get_logger("staubli_robot_driver::RobotDriver")),
  state_staleness_timeout_(0, 0),
  diagnostic_staleness_timeout_(0, 0)
{
    // Create sockets
    control_socket_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);
    diagnostics_socket_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);

    // Initialize variables
    set_state_staleness_timeout();
    set_diagnostic_staleness_timeout();
    reset_data();
}

RobotDriver::~RobotDriver()
{
}

bool RobotDriver::connect(const std::string& robot_ip, int timeout_ms)
{
    NetworkConfig default_config;
    default_config.robot_ip = robot_ip;
    return connect(default_config, timeout_ms);
}

bool RobotDriver::connect(const NetworkConfig& config, int timeout_ms)
{
    RCLCPP_INFO(logger_, "Connecting to Staubli robot...");

    // Timestamp for connection attempt
    rclcpp::Time start_connect_time = rclcpp::Clock().now();

    // Store the configuration
    network_config_ = config;

    // Check IP address (only non-default value)
    if (network_config_.robot_ip.empty()) {
        RCLCPP_ERROR(logger_, "Robot IP address is empty");
        return false;
    }

    // Disconnect if already connected
    if (control_socket_ && control_socket_->is_connected()) {
        RCLCPP_WARN(logger_, "Control socket is already connected, disconnecting first");
        if (!disconnect()) {
            RCLCPP_ERROR(logger_, "Failed to disconnect existing connection");
            return false;
        }
    }
    if (diagnostics_socket_ && diagnostics_socket_->is_connected()) {
        RCLCPP_WARN(logger_, "Diagnostics socket is already connected, disconnecting first");
        if (!disconnect()) {
            RCLCPP_ERROR(logger_, "Failed to disconnect existing connection");
            return false;
        }
    }

    // Connect control socket
    if (!control_socket_) {
        RCLCPP_ERROR(logger_, "Control socket is not initialized");
        return false;
    }
    if (!control_socket_->connect(
            network_config_.robot_ip,
            network_config_.control_port,
            network_config_.local_control_port))
    {
        RCLCPP_ERROR(
            logger_,
            "Failed to connect control socket to %s:%u",
            network_config_.robot_ip.c_str(),
            network_config_.control_port);
        return false;
    }
    RCLCPP_INFO(
        logger_,
        "Connected control socket to %s:%u",
        network_config_.robot_ip.c_str(),
        network_config_.control_port);

    // Create control interface
    control_interface_ = std::make_shared<
        RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>>(control_socket_);
    if (!control_interface_) {
        RCLCPP_ERROR(logger_, "Failed to create control interface");
        disconnect();
        return false;
    }

    // Connect diagnostics socket
    if (!diagnostics_socket_) {
        RCLCPP_ERROR(logger_, "Diagnostics socket is not initialized");
        disconnect();
        return false;
    }
    if (!diagnostics_socket_->connect(
            network_config_.robot_ip,
            network_config_.diagnostics_port,
            network_config_.local_diagnostics_port))
    {
        RCLCPP_ERROR(
            logger_,
            "Failed to connect diagnostics socket to %s:%u",
            network_config_.robot_ip.c_str(),
            network_config_.diagnostics_port);
        disconnect();
        return false;
    }
    RCLCPP_INFO(
        logger_,
        "Connected diagnostics socket to %s:%u",
        network_config_.robot_ip.c_str(),
        network_config_.diagnostics_port);

    // Create diagnostics subscriber
    diagnostics_subscriber_ = std::make_shared<
        RealTimeSocketSubscriber<DiagnosticDataMessage>>(diagnostics_socket_);
    if (!diagnostics_subscriber_) {
        RCLCPP_ERROR(logger_, "Failed to create diagnostics subscriber");
        disconnect();
        return false;
    }

    // Check interfaces readiness
    if (!control_interface_->is_ready())
    {
        RCLCPP_ERROR(logger_, "Control interface was not ready after connection");
        disconnect();
        return false;
    }
    if (!diagnostics_subscriber_->is_ready())
    {
        RCLCPP_ERROR(logger_, "Diagnostics interface was not ready after connection");
        disconnect();
        return false;
    }

    // Wait for the first valid messages
    bool cmd_interface_ready = false;
    bool diag_interface_ready = false;
    size_t number_of_tries = 0;
    RobotStateMessage robot_state_msg;
    DiagnosticDataMessage diagnostic_data_msg;
    while (rclcpp::Clock().now() - start_connect_time <
           rclcpp::Duration(std::chrono::milliseconds(timeout_ms))) {
        number_of_tries++;
        RCLCPP_DEBUG(logger_,
            "Waiting for valid messages from the robot... (try n°%zu)", number_of_tries);
        cmd_interface_ready = read_robot_state(robot_state_msg);
        diag_interface_ready = read_diagnostic_data(diagnostic_data_msg);
        if (!cmd_interface_ready) {
            RCLCPP_WARN(logger_, "State/Control interface not yet ready");
        }
        if (!diag_interface_ready) {
            RCLCPP_WARN(logger_, "Diagnostics interface not yet ready");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    bool interfaces_ready = cmd_interface_ready && diag_interface_ready;
    if (!interfaces_ready) {
        RCLCPP_ERROR_STREAM(
            logger_,
            "Failed to get valid messages from the robot within timeout: "
            << " after " << timeout_ms << " ms (" << number_of_tries << " tries)"
            << "\n  - Robot state message valid: " << (cmd_interface_ready? "yes":"no")
            << "\n  - Diagnostic message valid: " << (diag_interface_ready? "yes":"no"));
        // Abort connection
        disconnect();
        return false;
    }

    // Check protocol version
    if (interfaces_ready) {
        if (current_robot_state_.header.protocol_version != PROTOCOL_VERSION) {
            RCLCPP_ERROR(
                logger_,
                "Protocol version mismatch: driver version is %u, robot version is %u",
                PROTOCOL_VERSION,
                current_robot_state_.header.protocol_version);
            disconnect();
            return false;
        }
    }

    // Check robot compatibility
    if (interfaces_ready) {
        /*
        // Check controller type
        if (current_diagnostic_data_.robot_controller_type !=
            DiagnosticDataMessage::RobotControllerType::CS9)
        {
            RCLCPP_ERROR(
                logger_,
                "Unsupported robot controller type: only CS9 is supported, "
                "robot controller type is %u",
                static_cast<uint8_t>(current_diagnostic_data_.robot_controller_type));
            disconnect();
            return false;
        }
        */
        // Check firmware version
        RCLCPP_INFO_STREAM(
            logger_,
            "Robot firmware version: " <<
            current_diagnostic_data_.robot_firmware_version.major << "." <<
            current_diagnostic_data_.robot_firmware_version.minor << "." <<
            current_diagnostic_data_.robot_firmware_version.patch);
        // TODO(tpoignonec): add check for supported VAL3 versions
    }

    return interfaces_ready;
}

bool RobotDriver::disconnect()
{
    RCLCPP_INFO(logger_, "Disconnecting robot driver...");
    bool control_socket_disconnected = true;

    // Clean up control interface
    if (control_interface_) {
        control_interface_.reset();
    }

    // Clean up diagnostics subscriber
    if (diagnostics_subscriber_) {
        diagnostics_subscriber_.reset();
    }

    // Disconnect sockets
    if (control_socket_ && control_socket_->is_connected()) {
        if (control_socket_->is_receiving()) {
            if (!control_socket_->stop_receive_thread()) {
                RCLCPP_WARN(logger_, "Failed to stop receiving on control socket");
            }
        }
        control_socket_disconnected &= control_socket_->disconnect();
        if (!control_socket_disconnected) {
            RCLCPP_WARN(logger_, "Failed to disconnect control socket");
        }
    }

    bool diagnostics_socket_disconnected = true;
    if (diagnostics_socket_ && diagnostics_socket_->is_connected()) {
        if (diagnostics_socket_->is_receiving()) {
            if (!diagnostics_socket_->stop_receive_thread()) {
                RCLCPP_WARN(logger_, "Failed to stop receiving on diagnostics socket");
            }
        }
        diagnostics_socket_disconnected &= diagnostics_socket_->disconnect();
        if (!diagnostics_socket_disconnected) {
            RCLCPP_WARN(logger_, "Failed to disconnect diagnostics socket");
        }
    }

    // Reset state
    reset_data();

    return control_socket_disconnected && diagnostics_socket_disconnected;
}

bool RobotDriver::is_connected() const
{
    bool sockets_ok = control_socket_ && control_socket_->is_connected();
    sockets_ok &= diagnostics_socket_ && diagnostics_socket_->is_connected();

    bool interfaces_ok = control_interface_ && control_interface_->is_ready();
    interfaces_ok &= diagnostics_subscriber_ && diagnostics_subscriber_->is_ready();

    return sockets_ok && interfaces_ok;
}

bool RobotDriver::send_stop_all_command()
{
    RobotCommandMessage stop_command;
    if (!prepare_robot_command_message(stop_command, current_robot_state_)) {
        RCLCPP_ERROR(logger_, "Failed to prepare STOP (motion + IOs) command");
        return false;
    }
    // Send the command
    if (!control_interface_->send_message(stop_command)) {
        RCLCPP_ERROR(logger_, "Failed to send STOP (motion + IOs) command!");
        return false;
    }
    return true;
}

bool RobotDriver::send_robot_command(const RobotCommandMessage& command)
{
    if (!control_interface_ || !control_interface_->is_ready()) {
        RCLCPP_WARN(logger_, "Cannot send robot command: control interface not ready");
        return false;
    }

    // Check command validity
    if (command.command_type == CommandType::STOP)
    {
        bool all_zeros = true;
        for (auto& val : command.command_reference) {
            if (val != 0.0) {
                all_zeros = false;
                break;
            }
        }
        if (!all_zeros) {
            RCLCPP_WARN(logger_, "Robot command: STOP command should have zeroed reference!");
        }
    }

    // TODO(tpoignonec): add checks for position/velocity limits here?

    // Prepare the command message to send
    if (!control_interface_->send_message(command)) {
        RCLCPP_ERROR(logger_, "Failed to send robot command");
        return false;
    }
    return true;
}

bool RobotDriver::read_robot_state(RobotStateMessage& state)
{
    if (!control_interface_ || !control_interface_->is_ready()) {
        RCLCPP_WARN(logger_, "Cannot read robot state: control interface not ready");
        return false;
    }
    MessageStatus status;
    if (!control_interface_->read_message(current_robot_state_, status)) {
        RCLCPP_WARN(logger_, "Cannot read robot state: no message received");
        return false;
    }
    // Check message validity
    if (!is_state_msg_valid()) {
        RCLCPP_ERROR(logger_, "Cannot read robot state: message is not valid");
        return false;
    }
    // Check for staleness
    if (status.time_since_received >= state_staleness_timeout_) {
        RCLCPP_WARN(logger_, "Cannot read robot state: state message is stale (age: %.1f ms)",
            1e3 * status.time_since_received.seconds());
        return false;
    }
    // Check lost packages (warning only)
    if (status.lost_packages > 0) {
        RCLCPP_WARN(logger_, "Detected %zu lost state messages", status.lost_packages);
    }
    // Update the output state and last valid sequence ID, ONLY if valid
    state = current_robot_state_;
    last_valid_sequence_id_ = \
        static_cast<size_t>(current_robot_state_.header.sequence_number);
    return true;
}

bool RobotDriver::read_diagnostic_data(DiagnosticDataMessage& data)
{
    if (!diagnostics_subscriber_ || !diagnostics_subscriber_->is_ready()) {
        RCLCPP_WARN(logger_, "Cannot read diagnostic data: diagnostics subscriber not ready");
        return false;
    }
    MessageStatus status;
    if (!diagnostics_subscriber_->read_message(current_diagnostic_data_, status)) {
        RCLCPP_WARN(logger_, "Cannot read diagnostic data: no message received");
        return false;
    }
    // Check message validity
    if (!is_diagnostic_msg_valid()) {
        RCLCPP_ERROR(logger_, "Cannot read diagnostic data: message is not valid");
        return false;
    }
    // Check for staleness
    if (status.time_since_received >= diagnostic_staleness_timeout_) {
        RCLCPP_WARN(logger_, "Cannot read diagnostic data: message is stale (age: %.1f ms)",
            1e3 * status.time_since_received.seconds());
        return false;
    }
    // Check for lost packages (warning only)
    if (status.lost_packages > 0) {
        RCLCPP_WARN(logger_, "Detected %zu lost diagnostic messages", status.lost_packages);
    }
    // Update the output data ONLY if valid
    data = current_diagnostic_data_;
    return true;
}

void RobotDriver::set_state_staleness_timeout(rclcpp::Duration staleness_timeout)
{
    RCLCPP_INFO_STREAM(
        logger_,
        "Setting state staleness timeout to "
        << 1e-6 * staleness_timeout.nanoseconds() << " ms"
    );
    state_staleness_timeout_ = staleness_timeout;
}

/**
    * @brief Set the staleness timeout after which the diagnostic data is considered stale
    * @param staleness_timeout Timeout for staleness check; default is 100 ms.
    * @return True if timeout was set successfully, false otherwise
    */
void RobotDriver::set_diagnostic_staleness_timeout(rclcpp::Duration staleness_timeout)
{
    RCLCPP_INFO_STREAM(
        logger_,
        "Setting diagnostic staleness timeout to "
        << 1e-6 * staleness_timeout.nanoseconds() << " ms"
    );
    diagnostic_staleness_timeout_ = staleness_timeout;
}

/******************************************
 *            Helper methods              *
 *****************************************/

void RobotDriver::reset_data()
{
    last_valid_sequence_id_ = 0;

    // Invalidate current state and diagnostic data
    current_robot_state_.header.sequence_number = 0;
    current_robot_state_.header.message_type = static_cast<uint16_t>(MessageType::INVALID);
    current_diagnostic_data_.header.sequence_number = 0;
    current_diagnostic_data_.header.message_type = static_cast<uint16_t>(MessageType::INVALID);

    return;
}


bool RobotDriver::is_state_msg_valid() const
{
    return current_robot_state_.header.message_type ==
           static_cast<uint16_t>(MessageType::ROBOT_STATE);
}
bool RobotDriver::is_diagnostic_msg_valid() const
{
    return current_diagnostic_data_.header.message_type ==
           static_cast<uint16_t>(MessageType::DIAGNOSTIC_DATA);
}

}  // namespace staubli_robot_driver
