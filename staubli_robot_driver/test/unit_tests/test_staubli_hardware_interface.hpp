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

#pragma once

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

// ROS 2 includes
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "pluginlib/class_loader.hpp"

// Staubli driver includes
#include "staubli_robot_driver/mock_robot_server.hpp"
#include "staubli_robot_driver/staubli_hardware_interface.hpp"

using staubli_robot_driver::StaubliHardwareInterface;

// Tolerance for floating point comparisons, needed due to float32 casting
const double float32_precision = 1e-6;

/**
 * @brief Test fixture for StaubliHardwareInterface tests
 */
class StaubliHardwareInterfaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Start robot simulation
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  bool start_mock_robot() {
    mock_robot_ = std::make_unique<staubli_robot_driver::MockRobotServer>();
    staubli_robot_driver::MockRobotServer::NetworkConfig config;
    config.robot_ip = loopback_address_;
    config.control_port = robot_control_port_;
    config.local_control_port = ros2_control_port_;
    config.diagnostics_port = robot_diagnostics_port_;
    config.local_diagnostics_port = ros2_diagnostics_port_;
    if (!mock_robot_->init(config)) {
      mock_robot_.reset();
      std::cerr << "Failed to start mock robot" << std::endl;
      return false;
    }
    mock_robot_thread_ = std::thread([this]() {
      staubli_robot_driver::RobotStateMessage state;
      state.header.protocol_version = 1;
      state.has_joint_positions = true;
      state.has_joint_velocities = true;
      state.has_joint_torques = false;

      state.joint_positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
      state.joint_velocities = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06};

      for (size_t i = 0; i < 6; i++) {
        // On the robot-side, degrees are used for joint positions/velocities
        state.joint_positions[i] *= (180.0 / M_PI);
        state.joint_velocities[i] *= (180.0 / M_PI);
      }

      staubli_robot_driver::DiagnosticDataMessage diag;
      diag.header.sequence_number = 1;
      diag.header.protocol_version = 1;
      diag.robot_firmware_version = {1, 2, 3};
      diag.robot_controller_type = diag.RobotControllerType::CS9;

      while (rclcpp::ok() && !stop_mock_robot_.load()) {
        mock_robot_->send_mock_robot_state(state);
        mock_robot_->send_mock_diagnostic_data(diag);

        std::this_thread::sleep_for(std::chrono::milliseconds(4));  // 250 Hz
      }
    });
    return true;
  }

  void stop_mock_robot() {
    if (mock_robot_) {
      stop_mock_robot_.store(true);
      if (mock_robot_thread_.joinable()) {
        mock_robot_thread_.join();
      }
      mock_robot_->disconnect();
      mock_robot_.reset();
      stop_mock_robot_.store(false);
    }
  }

  /**
   * @brief Create a minimal hardware info for testing
   */
  hardware_interface::HardwareInfo createMinimalHardwareInfo()
  {
    hardware_interface::HardwareInfo info;
    info.name = "staubli_robot";
    info.type = "system";

    // Add some joints
    for (int i = 1; i <= 6; ++i) {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint_" + std::to_string(i);
      joint.type = "joint";

      // Add position command interface
      hardware_interface::InterfaceInfo pos_cmd;
      pos_cmd.name = "position";
      joint.command_interfaces.push_back(pos_cmd);

      // Add position and velocity state interfaces
      hardware_interface::InterfaceInfo pos_state;
      pos_state.name = "position";
      joint.state_interfaces.push_back(pos_state);

      hardware_interface::InterfaceInfo vel_state;
      vel_state.name = "velocity";
      joint.state_interfaces.push_back(vel_state);

      info.joints.push_back(joint);
    }

    // Add hardware parameters
    info.hardware_parameters["robot_ip"] = loopback_address_;
    info.hardware_parameters["control_port"] = std::to_string(robot_control_port_);
    info.hardware_parameters["diagnostic_port"] = std::to_string(robot_diagnostics_port_);
    info.hardware_parameters["local_control_port"] = std::to_string(ros2_control_port_);
    info.hardware_parameters["local_diagnostic_port"] = std::to_string(ros2_diagnostics_port_);
    info.hardware_parameters["num_digital_inputs"] = "16";
    info.hardware_parameters["num_digital_outputs"] = "16";
    info.hardware_parameters["num_analog_inputs"] = "2";
    info.hardware_parameters["num_analog_outputs"] = "2";

    return info;
  }

  /**
   * @brief Create hardware component interface params for new API
   */
  hardware_interface::HardwareComponentInterfaceParams createParams()
  {
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = createMinimalHardwareInfo();
    // For basic tests, we don't need to set executor or logger
    return params;
  }
  // Mock robot instance
  std::unique_ptr<staubli_robot_driver::MockRobotServer> mock_robot_;
  std::thread mock_robot_thread_;
  std::atomic<bool> stop_mock_robot_{false};

  // Robot configuration
  std::string loopback_address_ = "127.0.0.1";
  uint16_t robot_control_port_ = 8001;
  uint16_t robot_diagnostics_port_ = 8002;
  uint16_t ros2_control_port_ = 8003;
  uint16_t ros2_diagnostics_port_ = 8004;
};


bool read_state_interface(
  const std::vector<hardware_interface::StateInterface>& state_interfaces,
  const std::string& name, double& value)
{
  for (const auto& interface : state_interfaces) {
    if (interface.get_name() == name) {
      const auto opt_value = interface.get_optional<double>();
      if (!opt_value) {
        return false;
      }
      value = opt_value.value();
      return true;
    }
  }
  return false;
}
