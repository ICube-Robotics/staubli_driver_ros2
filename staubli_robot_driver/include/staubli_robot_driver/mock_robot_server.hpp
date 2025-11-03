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


#ifndef STAUBLI_ROBOT_DRIVER__MOCK_ROBOT_SERVER_HPP_
#define STAUBLI_ROBOT_DRIVER__MOCK_ROBOT_SERVER_HPP_

// C++
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Staubli Robot Driver
#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/real_time_socket_interface.hpp"
#include "staubli_robot_driver/communication/socket.hpp"
#include "rclcpp/rclcpp.hpp"

namespace staubli_robot_driver {

/**
 * @brief Mock robot server to simulate a Staubli robot for testing purposes
 *
 * @warning This class is a mock and does not implement all functionalities of the
 *          real Robot...
 */
class MockRobotServer {
public:
    /// Configuration structure for the communication
    struct NetworkConfig {
        /// IP address of the Staubli robot
        std::string robot_ip = "";
        /// Port for control communication on the robot
        uint16_t control_port = DEFAULT_CONTROL_PORT;
        /// Port for control communication on ROS2 side
        uint16_t local_control_port = DEFAULT_CONTROL_PORT;
        /// Port for diagnostics communication on the robot
        uint16_t diagnostics_port = DEFAULT_DIAGNOSTICS_PORT;
        /// Port for diagnostics communication on ROS2 side
        uint16_t local_diagnostics_port = DEFAULT_DIAGNOSTICS_PORT;
    };

public:
    /**
     * @brief Constructor
     */
    MockRobotServer();

    /**
     * @brief Destructor
     */
    ~MockRobotServer();


    /**
     * @brief Initialize the mock robot server with the given network configuration
     * @param config Network configuration
     * @return True if initialization was successful, false otherwise
     */
    bool init(const NetworkConfig& config);

    /**
     * @brief Check if mock robot server is initialized and ready
     * @return True if ready, false otherwise
     */
    bool is_ready() const;

    /**
     * @brief Disconnect the mock robot server
     * @return True if disconnected successfully, false otherwise
     */
    bool disconnect();

    /**
     * @brief Send a robot state message from the mock robot server
     *
     * @param state Mock robot state message to send
     * @return true if the message was sent successfully, false otherwise
     */
    bool send_mock_robot_state(const RobotStateMessage& state);

    /**
    * @brief Send a diagnostic data message from the mock robot server
    * @param data Mock diagnostic data message to send
    * @return true if the message was sent successfully, false otherwise
    */
    bool send_mock_diagnostic_data(const DiagnosticDataMessage& data);

    /**
     * @brief Receive a command message sent to the mock robot server
     *
     * @param command Output command message
     * @return True if a message was received successfully, false otherwise
     */
    bool get_last_received_command(RobotCommandMessage& command, MessageStatus& status);

private:
    // Logger
    rclcpp::Logger logger_;

    // Sockets
    NetworkConfig network_config_;
    std::shared_ptr<Socket> control_socket_;
    std::shared_ptr<Socket> diagnostics_socket_;

    // Communication interfaces
    std::shared_ptr<
        RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>> control_interface_;
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__MOCK_ROBOT_SERVER_HPP_
