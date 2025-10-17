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

#include "staubli_robot_driver/mock_robot_server.hpp"
#include "staubli_robot_driver/communication/socket.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

namespace staubli_robot_driver {

MockRobotServer::MockRobotServer()
: logger_(rclcpp::get_logger("staubli_robot_driver::MockRobotServer"))
{
    // Create sockets
    control_socket_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);
    diagnostics_socket_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);
}

MockRobotServer::~MockRobotServer()
{
    // Disconnect and cleanup
    if (control_socket_ && control_socket_->is_connected()) {
        control_socket_->disconnect();
    }
    if (diagnostics_socket_ && diagnostics_socket_->is_connected()) {
        diagnostics_socket_->disconnect();
    }
}

bool MockRobotServer::init(const NetworkConfig& config)
{
    RCLCPP_INFO(logger_, "Initializing mock robot server...");

    // Store the configuration
    network_config_ = config;

    // Check IP address
    if (network_config_.robot_ip.empty()) {
        RCLCPP_ERROR(logger_, "Robot IP address is empty");
        return false;
    }

    // Disconnect if already connected
    if (control_socket_ && control_socket_->is_connected()) {
        RCLCPP_WARN(logger_, "Control socket is already connected, disconnecting first");
        control_socket_->disconnect();
    }
    if (diagnostics_socket_ && diagnostics_socket_->is_connected()) {
        RCLCPP_WARN(logger_, "Diagnostics socket is already connected, disconnecting first");
        diagnostics_socket_->disconnect();
    }

    // Connect control socket (mock robot acts as server, so we "connect" to the client)
    if (!control_socket_) {
        RCLCPP_ERROR(logger_, "Control socket is not initialized");
        return false;
    }

    // For mock robot, we reverse the connection - mock robot connects to the driver's ports
    if (!control_socket_->connect(
            network_config_.robot_ip,
            network_config_.local_control_port,  // Connect to driver's local port
            network_config_.control_port))       // From mock robot's port
    {
        RCLCPP_ERROR(
            logger_,
            "Failed to connect mock robot control socket to %s:%u",
            network_config_.robot_ip.c_str(),
            network_config_.local_control_port);
        return false;
    }
    RCLCPP_INFO(
        logger_,
        "Connected mock robot control socket to %s:%u",
        network_config_.robot_ip.c_str(),
        network_config_.local_control_port);

    // Create control interface (mock robot receives commands and sends states)
    control_interface_ = std::make_shared<
        RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>>(control_socket_);
    if (!control_interface_) {
        RCLCPP_ERROR(logger_, "Failed to create mock robot control interface");
        return false;
    }

    // Connect diagnostics socket
    if (!diagnostics_socket_) {
        RCLCPP_ERROR(logger_, "Diagnostics socket is not initialized");
        return false;
    }

    if (!diagnostics_socket_->connect(
            network_config_.robot_ip,
            network_config_.local_diagnostics_port,  // Connect to driver's local port
            network_config_.diagnostics_port))       // From mock robot's port
    {
        RCLCPP_ERROR(
            logger_,
            "Failed to connect mock robot diagnostics socket to %s:%u",
            network_config_.robot_ip.c_str(),
            network_config_.local_diagnostics_port);
        return false;
    }
    RCLCPP_INFO(
        logger_,
        "Connected mock robot diagnostics socket to %s:%u",
        network_config_.robot_ip.c_str(),
        network_config_.local_diagnostics_port);

    // Check interfaces readiness
    if (!control_interface_->is_ready())
    {
        RCLCPP_WARN(logger_, "Control interface not immediately ready after connection");
        // Wait a bit for interface to become ready
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if (!control_interface_->is_ready()) {
            RCLCPP_ERROR(logger_, "Control interface was not ready after waiting");
            return false;
        }
    }

    RCLCPP_INFO(logger_, "Mock robot server initialized successfully");
    return true;
}

bool MockRobotServer::is_ready() const
{
    return control_interface_ && control_interface_->is_ready() &&
           diagnostics_socket_ && diagnostics_socket_->is_connected();
}

bool MockRobotServer::disconnect()
{
    RCLCPP_INFO(logger_, "Disconnecting mock robot server...");

    bool success = true;

    // Disconnect control socket
    if (control_socket_ && control_socket_->is_connected()) {
        if (!control_socket_->disconnect()) {
            RCLCPP_ERROR(logger_, "Failed to disconnect control socket");
            success = false;
        }
    }

    // Disconnect diagnostics socket
    if (diagnostics_socket_ && diagnostics_socket_->is_connected()) {
        if (!diagnostics_socket_->disconnect()) {
            RCLCPP_ERROR(logger_, "Failed to disconnect diagnostics socket");
            success = false;
        }
    }

    // Reset interfaces
    control_interface_.reset();

    if (success) {
        RCLCPP_INFO(logger_, "Mock robot server disconnected successfully");
    }

    return success;
}

bool MockRobotServer::send_mock_robot_state(const RobotStateMessage& state)
{
    if (!control_interface_ || !control_interface_->is_ready()) {
        RCLCPP_ERROR(logger_, "Control interface is not ready for sending robot state");
        return false;
    }

    // Send the robot state message
    if (!control_interface_->send_message(state)) {
        RCLCPP_ERROR(logger_, "Failed to send mock robot state message");
        return false;
    }

    RCLCPP_DEBUG(logger_, "Successfully sent mock robot state message");
    return true;
}

bool MockRobotServer::send_mock_diagnostic_data(const DiagnosticDataMessage& data)
{
    if (!diagnostics_socket_ || !diagnostics_socket_->is_connected()) {
        RCLCPP_ERROR(logger_, "Diagnostics socket is not connected");
        return false;
    }

    // Serialize the diagnostic data message
    std::vector<uint8_t> serialized_data(data.get_serialized_size());
    if (!data.serialize(serialized_data.data(), serialized_data.size())) {
        RCLCPP_ERROR(logger_, "Failed to serialize diagnostic data message");
        return false;
    }

    // Send the serialized data
    if (!diagnostics_socket_->send(serialized_data)) {
        RCLCPP_ERROR(logger_, "Failed to send diagnostic data message");
        return false;
    }

    RCLCPP_DEBUG(logger_, "Successfully sent mock diagnostic data message");
    return true;
}

bool MockRobotServer::get_last_received_command(RobotCommandMessage& command, MessageStatus& status)
{
    if (!control_interface_ || !control_interface_->is_ready()) {
        RCLCPP_ERROR(logger_, "Control interface is not ready for receiving commands");
        return false;
    }

    // Get the latest received message
    if (!control_interface_->read_message(command, status)) {
        RCLCPP_DEBUG(logger_, "No command message available");
        return false;
    }
    return true;
}

}  // namespace staubli_robot_driver
