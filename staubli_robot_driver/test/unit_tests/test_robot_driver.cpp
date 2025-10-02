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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <future>

// ROS 2 includes
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

// Staubli driver includes
#include "staubli_robot_driver/robot_driver.hpp"
#include "staubli_robot_driver/communication/socket.hpp"
#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/communication/protocol.hpp"
#include "staubli_robot_driver/real_time_socket_subscriber.hpp"
#include "staubli_robot_driver/real_time_socket_interface.hpp"

using staubli_robot_driver::RobotDriver;
using staubli_robot_driver::RobotCommandMessage;
using staubli_robot_driver::RobotStateMessage;
using staubli_robot_driver::DiagnosticDataMessage;
using staubli_robot_driver::CommandType;
using staubli_robot_driver::OperationMode;
using staubli_robot_driver::MessageStatus;
using staubli_robot_driver::RealTimeSocketSubscriber;
using staubli_robot_driver::RealTimeSocketInterface;

/**
 * @brief Test fixture for RobotDriver tests
 *
 * This test fixture provides the setup for testing the RobotDriver class:
 * 1. Creates mock server sockets for both control and diagnostic channels
 * 2. Configures the test environment with local ports and addresses
 * 3. Provides helper methods to send and receive messages to/from the robot driver
 */
class RobotDriverTest : public ::testing::Test {
    protected:
    /**
    * @brief Set up the test fixture
    *
    * Initializes the mock server sockets and test configuration
    */
    void SetUp() override {
    // Initialize ROS 2 logging
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // Create mock server sockets for control and diagnostic channels
    server_control_socket_ = staubli_robot_driver::SocketFactory::create(
        staubli_robot_driver::SocketFactory::ProtocolType::UDP);
    server_diagnostics_socket_ = staubli_robot_driver::SocketFactory::create(
        staubli_robot_driver::SocketFactory::ProtocolType::UDP);

    // Configure test ports
    loopback_address_ = "127.0.0.1";
    robot_control_port_ = 20021;       // Driver's local port for control
    server_control_port_ = 30021;      // Mock server's local port for control
    robot_diagnostics_port_ = 20022;   // Driver's local port for diagnostics
    server_diagnostics_port_ = 30022;  // Mock server's local port for diagnostics

    // Connect server sockets (simulating the robot controller)
    ASSERT_TRUE(server_control_socket_->connect(
        loopback_address_, robot_control_port_, server_control_port_));
    ASSERT_TRUE(server_diagnostics_socket_->connect(
        loopback_address_, robot_diagnostics_port_, server_diagnostics_port_));

    // Create RT interface for the mock server control connection
    // Note: Using inverted message types compared to the robot driver
    mock_control_interface_ = std::make_shared<
        RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>>(server_control_socket_);

    // Note: We're using the raw socket for diagnostics data since it's a one-way communication
    // and we don't have a dedicated publisher interface
    }

    bool start_mock_robot() {
        if (server_control_socket_ == nullptr || server_diagnostics_socket_ == nullptr) {
            std::cerr << "Server sockets are not initialized!" << std::endl;
            return false;
        }
        if (server_control_socket_->is_connected() == false ||
            server_diagnostics_socket_->is_connected() == false)
        {
            std::cerr << "Server sockets are not connected!" << std::endl;
            return false;
        }
        if (mock_control_interface_ == nullptr || !mock_control_interface_->is_ready()) {
            std::cerr << "Mock control interface is not ready!" << std::endl;
            return false;
        }
        stop_mock_robot_.store(false);
        mock_robot_all_ok_.store(true);
        mock_robot_thread_ = std::thread([this]() {
            static uint32_t sequence_number = 1;
            while (!stop_mock_robot_.load()) {
                bool all_ok = true;
                all_ok &= sendMockRobotState(sequence_number, false);  // Robot not in motion
                all_ok &= sendMockDiagnosticData(sequence_number);  // Send diag data too
                sequence_number++;
                if (!all_ok) {
                    std::cout << "Mock robot failed to send data!" << std::endl;
                    mock_robot_all_ok_.store(false);
                }
                // 4 ms robot controller cycle time
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }
            std::cout << "Mock robot thread stopping..." << std::endl;
        });
        return true;
    }

    bool stop_mock_robot() {
        stop_mock_robot_.store(true);
        if (mock_robot_thread_.joinable()) {
            mock_robot_thread_.join();
        }
        return true;
    }

    /**
    * @brief Tear down the test fixture
    *
    * Cleans up resources and ensures proper shutdown
    */
    void TearDown() override {
        // Clean up robot driver
        if (robot_driver_) {
            robot_driver_->disconnect();
            robot_driver_.reset();
        }

        // Clean up RT interface
        mock_control_interface_.reset();

        // Clean up server sockets
        if (server_control_socket_) {
            server_control_socket_->stop_receive_thread();
            server_control_socket_->disconnect();
        }
        if (server_diagnostics_socket_) {
            server_diagnostics_socket_->stop_receive_thread();
            server_diagnostics_socket_->disconnect();
        }
    }

    /**
    * @brief Create and send a robot state message from the mock server
    *
    * @param sequence_number The sequence ID for the message
    * @param in_motion Whether the robot is in motion
    * @return True if message was sent successfully
    */
    bool sendMockRobotState(uint32_t sequence_number, bool in_motion = false) {
        // Set message header
        sent_state_msg_.header.sequence_number = sequence_number;

        // Set robot state fields
        sent_state_msg_.operation_mode = OperationMode::AUTOMATIC;
        sent_state_msg_.power_on = true;
        sent_state_msg_.brakes_released = true;
        sent_state_msg_.motion_possible = true;
        sent_state_msg_.in_motion = in_motion;
        sent_state_msg_.error_state = false;

        // Set valid fields
        sent_state_msg_.has_joint_positions = true;
        sent_state_msg_.has_joint_velocities = true;

        // Set joint positions and velocities
        for (size_t i = 0; i < 6; i++) {
            sent_state_msg_.joint_positions[i] = 0.1 * i;
            sent_state_msg_.joint_velocities[i] = in_motion ? 0.01 * i : 0.0;
        }

        // Send using the RT interface
        return mock_control_interface_->send_message(sent_state_msg_);
    }
    /**
    * @brief Create and send a diagnostic message from the mock server
    *
    * This method sends diagnostic data directly through the raw socket.
    * We don't use a RealTimeSocketInterface here because:
    * 1. Diagnostic messages are one-way (robot to driver only)
    * 2. There's no dedicated publisher interface in the codebase
    * 3. The driver is using a RealTimeSocketSubscriber for this channel
    *
    * @param sequence_number The sequence ID for the message
    * @return True if message was sent successfully
    */
    bool sendMockDiagnosticData(uint32_t sequence_number) {
        // Set message header
        sent_diag_msg_.header.sequence_number = sequence_number;

        // Set diagnostic data
        sent_diag_msg_.robot_firmware_version = {1, 2, 3};  // Version 1.2.3
        // diag_msg.robot_controller_type = DiagnosticDataMessage::ControllerType::CS8;

        // Send the diagnostic data directly through the socket
        // (Since DiagnosticDataMessage is one-way we don't have a RT interface for sending it)
        std::vector<uint8_t> buffer(sent_diag_msg_.get_serialized_size());
        if (!sent_diag_msg_.serialize(buffer.data(), buffer.size())) {
            return false;
        }

        return server_diagnostics_socket_->send(buffer);
    }

    /**
    * @brief Wait for and receive a command message from the driver
    *
    * @param[out] cmd The received command message
    * @param timeout_ms Timeout in milliseconds
    * @return True if a message was received successfully
    */
    bool receiveMockCommand(RobotCommandMessage& cmd, int timeout_ms = 100) {
        // Set up a future to receive the message using the RT interface
        auto future = std::async(std::launch::async, [this, &cmd]() {
            MessageStatus status;
            return mock_control_interface_->read_message(cmd, status);
        });

        // Wait for the future to complete or timeout
        if (future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::timeout) {
            return false;
        }

        // Return the result from the future
        return future.get();
    }

    // Test fixture members
    std::unique_ptr<RobotDriver> robot_driver_;
    std::shared_ptr<staubli_robot_driver::Socket> server_control_socket_;
    std::shared_ptr<staubli_robot_driver::Socket> server_diagnostics_socket_;

    // RT communication interface for the mock server
    std::shared_ptr<RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>>
    mock_control_interface_;

    // Mock robot setup
    std::thread mock_robot_thread_;
    std::atomic<bool> stop_mock_robot_{false};
    std::atomic<bool> mock_robot_all_ok_{true};

    // Network configuration
    std::string loopback_address_;
    uint16_t robot_control_port_;
    uint16_t server_control_port_;
    uint16_t robot_diagnostics_port_;
    uint16_t server_diagnostics_port_;

    // Message instances for reuse
    RobotStateMessage sent_state_msg_;
    DiagnosticDataMessage sent_diag_msg_;
};

/**
 * @brief Test connection to the robot
 *
 * This test validates that the driver can:
 * - Successfully connect to a robot with proper network configuration
 * - Detect if it's connected correctly
 */
TEST_F(RobotDriverTest, ConnectionTest) {
    // First connect to the robot
    RobotDriver::NetworkConfig network_config;
    network_config.robot_ip = loopback_address_;
    network_config.control_port = server_control_port_;
    network_config.local_control_port = robot_control_port_;
    network_config.diagnostics_port = server_diagnostics_port_;
    network_config.local_diagnostics_port = robot_diagnostics_port_;

    // Create robot driver
    robot_driver_ = std::make_unique<RobotDriver>();
    ASSERT_NE(robot_driver_, nullptr);

    // Connect to the robot
    ASSERT_TRUE(start_mock_robot());

    EXPECT_TRUE(robot_driver_->connect(network_config, 1000 /* 1s timeout */));
    EXPECT_TRUE(robot_driver_->is_connected());

    // Stop the mock robot
    ASSERT_TRUE(stop_mock_robot());
    ASSERT_TRUE(mock_robot_all_ok_.load());
}

/**
 * @brief Test disconnection from the robot
 *
 * This test validates that the driver can:
 * - Successfully disconnect from a connected robot
 * - Report the disconnected state correctly
 */
TEST_F(RobotDriverTest, DisconnectionTest) {
    // First connect to the robot
    RobotDriver::NetworkConfig network_config;
    network_config.robot_ip = loopback_address_;
    network_config.control_port = server_control_port_;
    network_config.local_control_port = robot_control_port_;
    network_config.diagnostics_port = server_diagnostics_port_;
    network_config.local_diagnostics_port = robot_diagnostics_port_;

    // Start the mock robot
    ASSERT_TRUE(start_mock_robot());

    // Create robot driver
    robot_driver_ = std::make_unique<RobotDriver>();
    ASSERT_NE(robot_driver_, nullptr);

    // Connect to the robot
    ASSERT_TRUE(robot_driver_->connect(network_config, 1000 /* 1s timeout */));
    ASSERT_TRUE(robot_driver_->is_connected());

    // Now disconnect
    EXPECT_TRUE(robot_driver_->disconnect());

    // Verify disconnection
    EXPECT_FALSE(robot_driver_->is_connected());

    // Stop the mock robot
    ASSERT_TRUE(stop_mock_robot());
    ASSERT_TRUE(mock_robot_all_ok_.load());
}

/**
 * @brief Test reading robot state
 *
 * This test validates that the driver can:
 * - Successfully read robot state messages from the robot
 * - Parse the state correctly with all fields
 * - Detect staleness when messages aren't received regularly
 */
TEST_F(RobotDriverTest, ReadRobotStateTest) {
    // First connect to the robot
    RobotDriver::NetworkConfig network_config;
    network_config.robot_ip = loopback_address_;
    network_config.control_port = server_control_port_;
    network_config.local_control_port = robot_control_port_;
    network_config.diagnostics_port = server_diagnostics_port_;
    network_config.local_diagnostics_port = robot_diagnostics_port_;

    // Start the mock robot
    ASSERT_TRUE(start_mock_robot());

    // Create robot driver
    robot_driver_ = std::make_unique<RobotDriver>();
    ASSERT_NE(robot_driver_, nullptr);

    // Connect to the robot
    ASSERT_TRUE(robot_driver_->connect(network_config, 1000 /* 1s timeout */));
    ASSERT_TRUE(robot_driver_->is_connected());

    // Set staleness timeout
    robot_driver_->set_state_staleness_timeout(rclcpp::Duration(0, 1e8));  // 100ms

    // Read the robot state
    std::cout << "TEST : Reading robot state" << std::endl;
    RobotStateMessage received_state;
    ASSERT_TRUE(robot_driver_->read_robot_state(received_state));

    // Verify the state contents
    std::cout << "TEST : Verifying received state" << std::endl;
    EXPECT_EQ(received_state.operation_mode, sent_state_msg_.operation_mode);

    // Verify joint positions
    for (size_t i = 0; i < 6; i++) {
        EXPECT_NEAR(received_state.joint_positions[i], sent_state_msg_.joint_positions[i], 1e-6);
        EXPECT_NEAR(received_state.joint_velocities[i], sent_state_msg_.joint_velocities[i], 1e-6);
    }

    // STOP the mock robot to test staleness
    std::cout << "TEST : Stopping mock robot to test staleness detection" << std::endl;
    ASSERT_TRUE(stop_mock_robot());
    ASSERT_TRUE(mock_robot_all_ok_.load());

    // Test staleness detection - wait longer than staleness timeout
    std::this_thread::sleep_for(std::chrono::milliseconds(600));  // Wait longer than timeout

    // Try to read state again - should fail due to staleness
    RobotStateMessage stale_state;
    EXPECT_FALSE(robot_driver_->read_robot_state(stale_state));

    // Send a new state message to reset staleness
    std::cout << "TEST : Restarting mock robot to recover from staleness" << std::endl;
    ASSERT_TRUE(start_mock_robot());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // for some data to be sent

    // Should be able to read state again
    std::cout << "TEST : Reading robot state after staleness recovery" << std::endl;
    EXPECT_TRUE(robot_driver_->read_robot_state(received_state));

    // Stop the mock robot
    ASSERT_TRUE(stop_mock_robot());
    ASSERT_TRUE(mock_robot_all_ok_.load());
}

/**
 * @brief Main function that runs the tests
 *
 * Initializes Google Test and ROS 2
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
