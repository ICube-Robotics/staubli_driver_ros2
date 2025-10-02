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
#include <vector>

#include "staubli_robot_driver/communication/messages.hpp"

// Using declarations instead of namespace using-directive
using staubli_robot_driver::FrameHeader;
using staubli_robot_driver::RobotStateMessage;
using staubli_robot_driver::RobotCommandMessage;
using staubli_robot_driver::DiagnosticDataMessage;
using staubli_robot_driver::MessageType;
using staubli_robot_driver::OperationMode;
using staubli_robot_driver::CommandType;
using staubli_robot_driver::MAGIC_NUMBER;
using staubli_robot_driver::PROTOCOL_VERSION;

// Test fixture for message serialization/deserialization tests
class MessageSerializationTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize buffer
    buffer.resize(1024, 0);
  }

  // Buffer for serialization/deserialization
  std::vector<uint8_t> buffer;
};

// Test FrameHeader serialization/deserialization
TEST_F(MessageSerializationTest, FrameHeaderSerializationDeserialization) {
  FrameHeader original_header;
  original_header.magic_number = MAGIC_NUMBER;
  original_header.protocol_version = PROTOCOL_VERSION;
  original_header.message_type = static_cast<uint16_t>(MessageType::ROBOT_STATE);
  original_header.sequence_number = 42;
  original_header.payload_size = 256;

  // Serialize header
  ASSERT_TRUE(original_header.serialize(buffer.data(), buffer.size()));

  // Create a new header for deserialization
  FrameHeader deserialized_header;
  ASSERT_TRUE(deserialized_header.deserialize(buffer.data(), buffer.size()));

  // Verify all fields match
  EXPECT_EQ(original_header.magic_number, deserialized_header.magic_number);
  EXPECT_EQ(original_header.protocol_version, deserialized_header.protocol_version);
  EXPECT_EQ(original_header.message_type, deserialized_header.message_type);
  EXPECT_EQ(original_header.sequence_number, deserialized_header.sequence_number);
  EXPECT_EQ(original_header.payload_size, deserialized_header.payload_size);
}

// Test RobotStateMessage serialization/deserialization
TEST_F(MessageSerializationTest, RobotStateMessageSerializationDeserialization) {
  // Create and populate original message
  RobotStateMessage original_msg;

  // Set header fields
  original_msg.header.sequence_number = 42;

  // Set basic fields
  original_msg.operation_mode = OperationMode::AUTOMATIC;

  // Set status flags
  original_msg.power_on = true;
  original_msg.brakes_released = true;
  original_msg.motion_possible = true;
  original_msg.in_motion = false;
  original_msg.error_state = false;
  original_msg.wait_for_error_reset = false;
  original_msg.estop_pressed = false;
  original_msg.protective_stop = false;

  // Set valid fields flags
  original_msg.has_joint_positions = true;
  original_msg.has_joint_velocities = true;
  original_msg.has_joint_torques = false;
  original_msg.has_ft_sensor = true;
  original_msg.has_digital_inputs = true;
  original_msg.has_analog_inputs = false;

  // Set joint positions
  original_msg.joint_positions = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};

  // Set joint velocities
  original_msg.joint_velocities = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  // Set FT sensor readings
  original_msg.ft_sensor = {10.1, 20.2, 30.3, 40.4, 50.5, 60.6};

  // Set digital inputs
  original_msg.digital_inputs = {
    true, false, true, false,
    true, false, true, false,
    true, false, true, false,
    true, false, true, false
  };

  // Serialize message to buffer
  ASSERT_TRUE(original_msg.serialize(buffer.data(), buffer.size()));

  // Create new message for deserialization
  RobotStateMessage deserialized_msg;
  ASSERT_TRUE(deserialized_msg.deserialize(buffer.data(), buffer.size()));

  // Verify header fields
  EXPECT_EQ(original_msg.header.sequence_number, deserialized_msg.header.sequence_number);
  EXPECT_EQ(original_msg.header.message_type, deserialized_msg.header.message_type);

  // Verify basic fields
  EXPECT_EQ(original_msg.operation_mode, deserialized_msg.operation_mode);

  // Verify status flags
  EXPECT_EQ(original_msg.power_on, deserialized_msg.power_on);
  EXPECT_EQ(original_msg.brakes_released, deserialized_msg.brakes_released);
  EXPECT_EQ(original_msg.motion_possible, deserialized_msg.motion_possible);
  EXPECT_EQ(original_msg.in_motion, deserialized_msg.in_motion);
  EXPECT_EQ(original_msg.error_state, deserialized_msg.error_state);
  EXPECT_EQ(original_msg.wait_for_error_reset, deserialized_msg.wait_for_error_reset);
  EXPECT_EQ(original_msg.estop_pressed, deserialized_msg.estop_pressed);
  EXPECT_EQ(original_msg.protective_stop, deserialized_msg.protective_stop);

  // Verify valid fields flags
  EXPECT_EQ(original_msg.has_joint_positions, deserialized_msg.has_joint_positions);
  EXPECT_EQ(original_msg.has_joint_velocities, deserialized_msg.has_joint_velocities);
  EXPECT_EQ(original_msg.has_joint_torques, deserialized_msg.has_joint_torques);
  EXPECT_EQ(original_msg.has_ft_sensor, deserialized_msg.has_ft_sensor);
  EXPECT_EQ(original_msg.has_digital_inputs, deserialized_msg.has_digital_inputs);
  EXPECT_EQ(original_msg.has_analog_inputs, deserialized_msg.has_analog_inputs);

  // Verify joint positions
  for (size_t i = 0; i < original_msg.joint_positions.size(); ++i) {
    EXPECT_NEAR(original_msg.joint_positions[i], deserialized_msg.joint_positions[i], 1e-5);
  }

  // Verify joint velocities
  for (size_t i =.0; i < original_msg.joint_velocities.size(); ++i) {
    EXPECT_NEAR(original_msg.joint_velocities[i], deserialized_msg.joint_velocities[i], 1e-5);
  }

  // Verify FT sensor readings
  for (size_t i = 0; i < original_msg.ft_sensor.size(); ++i) {
    EXPECT_NEAR(original_msg.ft_sensor[i], deserialized_msg.ft_sensor[i], 1e-5);
  }

  // Verify digital inputs
  for (size_t i = 0; i < original_msg.digital_inputs.size(); ++i) {
    EXPECT_EQ(original_msg.digital_inputs[i], deserialized_msg.digital_inputs[i]);
  }
}

// Test RobotCommandMessage serialization/deserialization
TEST_F(MessageSerializationTest, RobotCommandMessageSerializationDeserialization) {
  // Create and populate original message
  RobotCommandMessage original_msg;

  // Set header fields
  original_msg.header.sequence_number = 99;

  // Set command fields
  original_msg.command_type = CommandType::JOINT_POSITION;
  original_msg.command_reference = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};

  // Set digital outputs
  original_msg.digital_outputs = {
    false, true, false, true,
    false, true, false, true,
    false, true, false, true,
    false, true, false, true
  };

  // Set analog outputs
  original_msg.analog_outputs = {5.5, 6.6, 7.7, 8.8};

  // Serialize message to buffer
  ASSERT_TRUE(original_msg.serialize(buffer.data(), buffer.size()));

  // Create new message for deserialization
  RobotCommandMessage deserialized_msg;
  ASSERT_TRUE(deserialized_msg.deserialize(buffer.data(), buffer.size()));

  // Verify header fields
  EXPECT_EQ(original_msg.header.sequence_number, deserialized_msg.header.sequence_number);
  EXPECT_EQ(original_msg.header.message_type, deserialized_msg.header.message_type);

  // Verify command fields
  EXPECT_EQ(original_msg.command_type, deserialized_msg.command_type);

  // Verify command reference
  for (size_t i = 0; i < original_msg.command_reference.size(); ++i) {
    EXPECT_NEAR(original_msg.command_reference[i], deserialized_msg.command_reference[i], 1e-5);
  }

  // Verify digital outputs
  for (size_t i = 0; i < original_msg.digital_outputs.size(); ++i) {
    EXPECT_EQ(original_msg.digital_outputs[i], deserialized_msg.digital_outputs[i]);
  }

  // Verify analog outputs
  for (size_t i = 0; i < original_msg.analog_outputs.size(); ++i) {
    EXPECT_NEAR(original_msg.analog_outputs[i], deserialized_msg.analog_outputs[i], 1e-5);
  }
}

// Test DiagnosticDataMessage serialization/deserialization
TEST_F(MessageSerializationTest, DiagnosticDataMessageSerializationDeserialization) {
  // Create and populate original message
  DiagnosticDataMessage original_msg;

  // Set header fields
  original_msg.header.sequence_number = 123;

  // Set diagnostic fields
  original_msg.robot_controller_type = DiagnosticDataMessage::RobotControllerType::CS9;
  original_msg.robot_firmware_version = {4, 2, 1};  // v4.2.1

  // Set error log entries
  for (size_t i = 0; i < original_msg.error_log.size(); ++i) {
    original_msg.error_log[i].timestamp = 1000 * (i + 1);
    original_msg.error_log[i].error_code = 100 + i;
  }

  // Set control task status
  original_msg.control_task_status = 0xABCD;

  // Serialize message to buffer
  ASSERT_TRUE(original_msg.serialize(buffer.data(), buffer.size()));

  // Create new message for deserialization
  DiagnosticDataMessage deserialized_msg;
  ASSERT_TRUE(deserialized_msg.deserialize(buffer.data(), buffer.size()));

  // Verify header fields
  EXPECT_EQ(original_msg.header.sequence_number, deserialized_msg.header.sequence_number);
  EXPECT_EQ(original_msg.header.message_type, deserialized_msg.header.message_type);

  // Verify diagnostic fields
  EXPECT_EQ(original_msg.robot_controller_type, deserialized_msg.robot_controller_type);
  EXPECT_EQ(original_msg.robot_firmware_version.major,
    deserialized_msg.robot_firmware_version.major);
  EXPECT_EQ(original_msg.robot_firmware_version.minor,
    deserialized_msg.robot_firmware_version.minor);
  EXPECT_EQ(original_msg.robot_firmware_version.patch,
    deserialized_msg.robot_firmware_version.patch);

  // Verify error log entries
  for (size_t i = 0; i < original_msg.error_log.size(); ++i) {
    EXPECT_EQ(original_msg.error_log[i].timestamp,
        deserialized_msg.error_log[i].timestamp);
    EXPECT_EQ(original_msg.error_log[i].error_code,
        deserialized_msg.error_log[i].error_code);
  }

  // Verify control task status
  EXPECT_EQ(original_msg.control_task_status, deserialized_msg.control_task_status);
}

// Test std::vector-based serialization/deserialization for RobotStateMessage
TEST_F(MessageSerializationTest, RobotStateMessageVectorSerialization) {
  // Create and populate original message
  RobotStateMessage original_msg;

  // Set basic fields
  original_msg.operation_mode = OperationMode::REMOTE;
  original_msg.power_on = true;
  original_msg.has_joint_positions = true;
  original_msg.joint_positions = {-0.1, -0.2, -0.3, -0.4, -0.5, -0.6};

  // Serialize to vector

  // Try to serialize with insufficient buffer size, should fail
  std::vector<uint8_t> data_too_small(10);
  ASSERT_FALSE(original_msg.serialize(data_too_small));

  std::vector<uint8_t> data;
  data.resize(original_msg.get_serialized_size());
  ASSERT_TRUE(original_msg.serialize(data));

  // Create new message for deserialization
  RobotStateMessage deserialized_msg;
  ASSERT_TRUE(deserialized_msg.deserialize(data));

  // Verify fields
  EXPECT_EQ(original_msg.operation_mode, deserialized_msg.operation_mode);
  EXPECT_EQ(original_msg.power_on, deserialized_msg.power_on);
  EXPECT_EQ(original_msg.has_joint_positions, deserialized_msg.has_joint_positions);

  for (size_t i = 0; i < original_msg.joint_positions.size(); ++i) {
    EXPECT_NEAR(original_msg.joint_positions[i],
        deserialized_msg.joint_positions[i], 1e-5);
  }
}

// Test buffer size validation for serialization/deserialization
TEST_F(MessageSerializationTest, BufferSizeValidation) {
  RobotStateMessage msg;

  // Try to serialize with insufficient buffer size
  std::vector<uint8_t> small_buffer(10);
  EXPECT_FALSE(msg.serialize(small_buffer.data(), small_buffer.size()));

  // Try to deserialize with insufficient buffer size
  EXPECT_FALSE(msg.deserialize(small_buffer.data(), small_buffer.size()));
}

// Main function to run the tests
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
