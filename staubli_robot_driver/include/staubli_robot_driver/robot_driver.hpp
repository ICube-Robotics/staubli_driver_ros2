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


#ifndef STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HPP_
#define STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/real_time_socket_interface.hpp"

namespace staubli_robot_driver {

/**
 * @brief Main interface for controlling Staubli robots
 *
 * This class provides methods to connect to a Staubli robot,
 * send commands, and receive state and diagnostic information.
 */
class RobotDriver {
public:
    /// Configuration structure for the communication
    struct NetworkConfig {
        /// IP address of the Staubli robot
        std::string robot_ip = "";
        /// Local IP address to bind to (empty string binds to any interface)
        std::string local_ip = "";
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
    RobotDriver();

    /**
     * @brief Destructor
     */
    ~RobotDriver();

    /**
     * @brief Connect to the robot with the default network configuration
     * @param robot_ip IP address of the robot
     * @param timeout_ms Connection timeout in milliseconds
     * @return True if connected successfully, false otherwise
     */
    bool connect(const std::string& robot_ip, int timeout_ms);

    /**
     * @brief Connect to the robot with a specific network configuration
     * @param config Network configuration
     * @param timeout_ms Connection timeout in milliseconds
     * @return True if connected successfully, false otherwise
     */
    bool connect(const NetworkConfig& config, int timeout_ms = 5000);

    /**
     * @brief Disconnect from the robot
     * @return True if disconnected successfully, false otherwise
     */
    bool disconnect();

    /**
     * @brief Check if connected to the robot (does not check for message staleness)
     * @return True if connected, false otherwise
     */
    bool is_connected() const;

    /**
     * @brief Send a STOP command to the robot (halts motion and sets IOs to zero)
     * @return True if command was sent successfully, false otherwise
     */
    bool send_stop_all_command();

    /**
     * @brief Send a command to the robot
     *
     * @param command Command to send
     * @return True if command was valid and sent successfully, false otherwise
     */
    bool send_robot_command(const RobotCommandMessage& command);

    /**
     * @brief Get the current (latest) robot state
     *
     * @note This should be called at a regular interval (e.g. in a control loop).
     *       The method will update the internal state while checking for staleness.
     *       If the state is stale, this method will return false. In that case, the user
     *       should take appropriate action (e.g. stop the robot) based on previous readings only.
     *
     * @param[out] state Robot state
     * @return True if state was successfully retrieved, false otherwise
     */
    bool read_robot_state(RobotStateMessage& state);

    /**
     * @brief Get the latest diagnostic data, see `update_robot_state()` for notes
     * @param[out] data Diagnostic data
     * @return True if data was successfully retrieved, false otherwise
     */
    bool read_diagnostic_data(DiagnosticDataMessage& data);

    /**
     * @brief Set the staleness timeout after which the robot state is considered stale
     * @param staleness_timeout Timeout for staleness check; default is 10 ms.
     * @return True if timeout was set successfully, false otherwise
     */
    void set_state_staleness_timeout(
        rclcpp::Duration staleness_timeout = rclcpp::Duration(0, 1e7));

    /**
     * @brief Set the staleness timeout after which the diagnostic data is considered stale
     * @param staleness_timeout Timeout for staleness check; default is 100 ms.
     * @return True if timeout was set successfully, false otherwise
     */
    void set_diagnostic_staleness_timeout(
        rclcpp::Duration staleness_timeout = rclcpp::Duration(0, 1e8));

    /**
     * @brief Get the sequence ID of the last valid received robot state message
     * @return Sequence ID of the last valid robot state message
     */
    size_t get_current_message_sequence() const {
        return last_valid_sequence_id_;
    }

protected:
    // Helper methods
    void reset_data();
    bool is_state_msg_valid() const;
    bool is_diagnostic_msg_valid() const;

private:
    // Logger
    rclcpp::Logger logger_;

    // Sockets
    NetworkConfig network_config_;
    std::shared_ptr<Socket> control_socket_;
    std::shared_ptr<Socket> diagnostics_socket_;

    // Communication interfaces
    std::shared_ptr<
        RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>> control_interface_;
    std::shared_ptr<
        RealTimeSocketSubscriber<DiagnosticDataMessage>> diagnostics_subscriber_;

    // State tracking
    size_t last_valid_sequence_id_ = 0;
    RobotStateMessage current_robot_state_;
    DiagnosticDataMessage current_diagnostic_data_;

    // Staleness timeouts
    rclcpp::Duration state_staleness_timeout_;
    rclcpp::Duration diagnostic_staleness_timeout_;
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HPP_
