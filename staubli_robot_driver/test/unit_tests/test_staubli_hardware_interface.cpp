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

#include "test_staubli_hardware_interface.hpp"

/**
 * @brief Test that the hardware interface can be constructed
 */
TEST_F(StaubliHardwareInterfaceTest, Construction)
{
  EXPECT_NO_THROW({
    auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  });
}

/**
 * @brief Test that the hardware interface can be initialized with minimal config
 */
TEST_F(StaubliHardwareInterfaceTest, Initialization)
{
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  auto params = createParams();

  auto result = hardware_interface->on_init(params);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

/**
 * @brief Test that initialization fails with empty hardware info
 */
TEST_F(StaubliHardwareInterfaceTest, InitializationFailsWithEmptyInfo)
{
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  hardware_interface::HardwareComponentInterfaceParams params;
  // Empty hardware_info should cause initialization to fail

  auto result = hardware_interface->on_init(params);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

/**
 * @brief Test hardware interface configuration
 */
TEST_F(StaubliHardwareInterfaceTest, Configuration)
{
  // Start mock robot
  ASSERT_TRUE(start_mock_robot());
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Instance of hardware interface
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  auto params = createParams();

  // Initialize first
  auto init_result = hardware_interface->on_init(params);
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  // Configuration would fail without actual robot
  rclcpp_lifecycle::State previous_state;
  auto config_result = hardware_interface->on_configure(previous_state);
  EXPECT_EQ(config_result, hardware_interface::CallbackReturn::SUCCESS);

  // Stop mock robot
  stop_mock_robot();
}

/**
 * @brief Test that the hardware interface configuration fails without an actual robot
 */
TEST_F(StaubliHardwareInterfaceTest, ConfigurationShouldFailWithoutRobot)
{
  // This test is disabled because it requires an actual robot connection
  // In a real deployment, the robot would need to be available
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  auto params = createParams();

  // Initialize first
  auto init_result = hardware_interface->on_init(params);
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  // Configuration would fail without actual robot
  rclcpp_lifecycle::State previous_state;
  auto config_result = hardware_interface->on_configure(previous_state);
  EXPECT_EQ(config_result, hardware_interface::CallbackReturn::ERROR);
}

/**
 * @brief Test plugin loading via pluginlib
 */
TEST_F(StaubliHardwareInterfaceTest, PluginLoading)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  EXPECT_NO_THROW({
    auto hardware_interface = loader.createSharedInstance(
      "staubli_robot_driver/StaubliHardwareInterface");
    EXPECT_NE(hardware_interface, nullptr);
  });
}

/**
 * @brief Test plugin loading and initialization via pluginlib
 */
TEST_F(StaubliHardwareInterfaceTest, PluginLoadingAndInitialization)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");

  auto hardware_interface = loader.createSharedInstance(
    "staubli_robot_driver/StaubliHardwareInterface");
  ASSERT_NE(hardware_interface, nullptr);

  auto params = createParams();
  auto result = hardware_interface->on_init(params);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

/**
 * @brief Test command and state interface export
 */
TEST_F(StaubliHardwareInterfaceTest, InterfaceExport)
{
  // Start mock robot
  ASSERT_TRUE(start_mock_robot());
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Initialize & configure
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  auto params = createParams();
  auto init_result = hardware_interface->on_init(params);
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  rclcpp_lifecycle::State previous_state;
  auto config_result = hardware_interface->on_configure(previous_state);
  ASSERT_EQ(config_result, hardware_interface::CallbackReturn::SUCCESS);

  // Export command interfaces
  auto command_interfaces = hardware_interface->export_command_interfaces();
  // 6 joints * [ 4 : 3 interfaces (pos,vel,eff) + 1 interface for ACC (limits only) ]
  // + 16 digital outputs + 2 analog outputs = 42
  EXPECT_EQ(command_interfaces.size(), 42);

  if (command_interfaces.size() != 42) {
    std::cerr << "Command interfaces:" << std::endl;
    for (const auto& interface : command_interfaces) {
      std::cerr << " - " << interface.get_name() << std::endl;
    }
  }

  // Export state interfaces
  auto state_interfaces = hardware_interface->export_state_interfaces();
  // 6 joints * 3 interfaces (18)
  // + 16 digital inputs + 2 analog inputs (18)
  // + 10 supervisory interfaces (robot mode, error code, etc.)
  //       = 46
  // If we use IO cmd feedback :
  // + 16 digital outputs + 2 analog outputs
  //       = 64
  EXPECT_EQ(state_interfaces.size(), 46);

  if (state_interfaces.size() != 46) {
    std::cerr << "State interfaces:" << std::endl;
    for (const auto& interface : state_interfaces) {
      std::cerr << " - " << interface.get_name() << std::endl;
    }
  }

  // Check joint interface names
  bool found_joint1_pos_cmd = false;
  bool found_joint1_vel_cmd = false;
  bool found_joint1_pos_state = false;
  bool found_joint1_vel_state = false;

  for (const auto& interface : command_interfaces) {
    if (interface.get_name() == "joint_1/position") {
      found_joint1_pos_cmd = true;
    }
    if (interface.get_name() == "joint_1/velocity") {
      found_joint1_vel_cmd = true;
    }
  }

  for (const auto& interface : state_interfaces) {
    if (interface.get_name() == "joint_1/position") {
      found_joint1_pos_state = true;
    }
    if (interface.get_name() == "joint_1/velocity") {
      found_joint1_vel_state = true;
    }
  }

  EXPECT_TRUE(found_joint1_pos_cmd);
  EXPECT_TRUE(found_joint1_vel_cmd);
  EXPECT_TRUE(found_joint1_pos_state);
  EXPECT_TRUE(found_joint1_vel_state);

  // Check the supervision GPIOs (not exhaustive...)
  std::string supervision_gpio_name = "supervision";
  bool found_operation_mode = false;
  bool found_operation_mode_status = false;
  bool found_safety_status = false;


  for (const auto& interface : state_interfaces) {
    if (interface.get_name() == supervision_gpio_name + "/operation_mode") {
      found_operation_mode = true;
    }
    if (interface.get_name() == supervision_gpio_name + "/operation_mode_status") {
      found_operation_mode_status = true;
    }
    if (interface.get_name() == supervision_gpio_name + "/safety_status") {
      found_safety_status = true;
    }
  }

  EXPECT_TRUE(found_operation_mode);
  EXPECT_TRUE(found_operation_mode_status);
  EXPECT_TRUE(found_safety_status);

  // Stop mock robot
  stop_mock_robot();
}

/**
 * @brief Test lifecycle state transitions
 */
TEST_F(StaubliHardwareInterfaceTest, LifecycleTransitions)
{
  // Start mock robot in the background
  ASSERT_TRUE(start_mock_robot());
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // This test is disabled because it requires an actual robot connection
  auto hardware_interface = std::make_unique<StaubliHardwareInterface>();
  auto params = createParams();

  // Initialize
  auto init_result = hardware_interface->on_init(params);
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  // Configure
  rclcpp_lifecycle::State previous_state;
  auto config_result = hardware_interface->on_configure(previous_state);
  EXPECT_EQ(config_result, hardware_interface::CallbackReturn::SUCCESS);

  // Activate
  auto activate_result = hardware_interface->on_activate(previous_state);
  EXPECT_EQ(activate_result, hardware_interface::CallbackReturn::SUCCESS);

  // Read should work
  auto current_time = rclcpp::Time(0, 1000);
  auto period = rclcpp::Duration(0, 4 * 1e6);  // 4 ms
  auto read_result = hardware_interface->read(current_time, period);
  EXPECT_EQ(read_result, hardware_interface::return_type::OK);

  // Check that state interfaces have been updated
  double joint1_pos = 0.0;
  bool found = read_state_interface(hardware_interface->export_state_interfaces(),
    "joint_1/position", joint1_pos);
  EXPECT_TRUE(found);
  EXPECT_NEAR(joint1_pos, 0.1, float32_precision);  // Position interface should have been updated

  double joint1_vel = 0.0;
  found = read_state_interface(hardware_interface->export_state_interfaces(),
    "joint_1/velocity", joint1_vel);
  EXPECT_TRUE(found);
  EXPECT_NEAR(joint1_vel, 0.01, float32_precision);  // Velocity interface should have been updated

  // Update command interfaces to some values
  for (auto& cmd_interface : hardware_interface->export_command_interfaces()) {
    if (cmd_interface.get_name().find("position") != std::string::npos) {
      ASSERT_TRUE(cmd_interface.set_value(1.0));
    }
  }

  // Write should work
  auto write_result = hardware_interface->write(current_time, period);
  EXPECT_EQ(write_result, hardware_interface::return_type::OK);

  // Stop mock robot
  stop_mock_robot();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
