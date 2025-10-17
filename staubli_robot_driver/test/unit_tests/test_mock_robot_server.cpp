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

// ROS 2 includes
#include "rclcpp/rclcpp.hpp"

// Staubli driver includes
#include "staubli_robot_driver/mock_robot_server.hpp"
#include "staubli_robot_driver/communication/messages.hpp"

using staubli_robot_driver::MessageStatus;
using staubli_robot_driver::MockRobotServer;
using staubli_robot_driver::RobotStateMessage;
using staubli_robot_driver::RobotCommandMessage;
using staubli_robot_driver::DiagnosticDataMessage;

/**
 * @brief Test fixture for MockRobotServer tests
 */
class MockRobotTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  /**
   * @brief Create a network configuration for testing
   */
  MockRobotServer::NetworkConfig createTestConfig()
  {
    MockRobotServer::NetworkConfig config;
    config.robot_ip = "127.0.0.1";
    config.control_port = staubli_robot_driver::DEFAULT_CONTROL_PORT;
    config.local_control_port = staubli_robot_driver::DEFAULT_CONTROL_PORT;
    config.diagnostics_port = staubli_robot_driver::DEFAULT_DIAGNOSTICS_PORT;
    config.local_diagnostics_port = staubli_robot_driver::DEFAULT_DIAGNOSTICS_PORT;
    return config;
  }

  /**
   * @brief Create a sample robot state message
   */
  RobotStateMessage createSampleRobotState()
  {
    RobotStateMessage state;
    state.header.sequence_number = 1;
    state.header.protocol_version = 1;

    state.has_joint_positions = true;
    state.has_joint_velocities = true;
    state.has_joint_torques = false;

    for (int i = 0; i < 6; ++i) {
      state.joint_positions[i] = i * 0.1;
      state.joint_velocities[i] = i * 0.01;
    }

    return state;
  }

  /**
   * @brief Create a sample diagnostic data message
   */
  DiagnosticDataMessage createSampleDiagnosticData()
  {
    DiagnosticDataMessage diag;
    diag.header.sequence_number = 1;
    diag.header.protocol_version = 1;

    diag.robot_firmware_version = {1, 2, 3};

    diag.robot_controller_type = diag.RobotControllerType::CS9;
    diag.error_log[0].error_code = 1234;
    diag.error_log[0].timestamp = 5678;

    return diag;
  }
};

/**
 * @brief Test that MockRobotServer can be constructed
 */
TEST_F(MockRobotTest, Construction)
{
  EXPECT_NO_THROW({
    auto mock_robot = std::make_unique<MockRobotServer>();
  });
}

/**
 * @brief Test MockRobotServer initialization
 */
TEST_F(MockRobotTest, Initialization)
{
  auto mock_robot = std::make_unique<MockRobotServer>();
  auto config = createTestConfig();

  // Note: This test may fail if ports are already in use
  // In a real test environment, you'd want to use available ports
  EXPECT_NO_THROW({
     bool result = mock_robot->init(config);
     ASSERT_TRUE(result);
  });
}

/**
 * @brief Test MockRobotServer disconnect
 */
TEST_F(MockRobotTest, Disconnect)
{
  auto mock_robot = std::make_unique<MockRobotServer>();

  // Should be able to disconnect even if not connected
  EXPECT_NO_THROW({
    bool result = mock_robot->disconnect();
    ASSERT_TRUE(result);
  });
}

/**
 * @brief Test MockRobotServer is_ready method
 */
TEST_F(MockRobotTest, IsReady)
{
  auto mock_robot = std::make_unique<MockRobotServer>();

  // Should not be ready initially
  EXPECT_FALSE(mock_robot->is_ready());
}

/**
 * @brief Test sending robot state without initialization
 */
TEST_F(MockRobotTest, SendRobotStateWithoutInit)
{
  auto mock_robot = std::make_unique<MockRobotServer>();
  auto state = createSampleRobotState();

  // Should fail because not initialized
  EXPECT_FALSE(mock_robot->send_mock_robot_state(state));
}

/**
 * @brief Test sending diagnostic data without initialization
 */
TEST_F(MockRobotTest, SendDiagnosticDataWithoutInit)
{
  auto mock_robot = std::make_unique<MockRobotServer>();
  auto diag = createSampleDiagnosticData();

  // Should fail because not initialized
  EXPECT_FALSE(mock_robot->send_mock_diagnostic_data(diag));
}

/**
 * @brief Test getting command without initialization
 */
TEST_F(MockRobotTest, GetCommandWithoutInit)
{
  auto mock_robot = std::make_unique<MockRobotServer>();
  RobotCommandMessage command;
  MessageStatus status;
  // Should fail because not initialized
  EXPECT_FALSE(mock_robot->get_last_received_command(command, status));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
