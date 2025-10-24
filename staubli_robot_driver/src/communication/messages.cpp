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


#include <arpa/inet.h>  // For htonl, htons, etc.

#include <cstring>
#include <iostream>
#include <stdexcept>

#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/communication/serialization.hpp"

namespace staubli_robot_driver {

// RobotState implementation

RobotStateMessage::RobotStateMessage() : Message()
{
    // Initialize header
    header.message_type = static_cast<uint8_t>(MessageType::ROBOT_STATE);
    header.payload_size = static_cast<uint16_t>(
        get_serialized_size() - FrameHeader::get_serialized_size());
    header.protocol_version = PROTOCOL_VERSION;
    header.magic_number = MAGIC_NUMBER;
    header.sequence_number = 0;
    // Sequence delay
    sequence_delay = 1000;
    // Initialize measurements
    joint_positions.fill(0.0);
    joint_velocities.fill(0.0);
    joint_torques.fill(0.0);
    ft_sensor.fill(0.0);
    digital_inputs.fill(false);
    analog_inputs.fill(0.0);
}

size_t RobotStateMessage::message_size() { return 132; }

bool RobotStateMessage::serialize(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "RobotState::serialize: "
            << "Buffer size is too small for RobotState" << std::endl;
        return false;
    }

    // Assemble status flags
    uint16_t status_flags = 0;
    if (power_on)     status_flags |= static_cast<uint16_t>(StatusFlag::POWER_ON);
    if (brakes_released) status_flags |= static_cast<uint16_t>(StatusFlag::BRAKES_RELEASED);
    if (motion_possible) status_flags |= static_cast<uint16_t>(StatusFlag::MOTION_POSSIBLE);
    if (in_motion)       status_flags |= static_cast<uint16_t>(StatusFlag::IN_MOTION);
    if (error_state)     status_flags |= static_cast<uint16_t>(StatusFlag::ERROR_STATE);
    if (wait_for_error_reset) status_flags |= \
        static_cast<uint16_t>(StatusFlag::WAIT_FOR_ERROR_RESET);
    if (estop_pressed)   status_flags |= static_cast<uint16_t>(StatusFlag::ESTOP_PRESSED);
    if (protective_stop) status_flags |= static_cast<uint16_t>(StatusFlag::PROTECTIVE_STOP);

    // Assemble valid fields flags
    uint16_t valid_fields_flags = 0;
    if (has_joint_positions)  valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::JOINT_POSITIONS);
    if (has_joint_velocities) valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::JOINT_VELOCITIES);
    if (has_joint_torques)    valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::JOINT_TORQUES);
    if (has_ft_sensor)        valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::FT_SENSOR);
    if (has_digital_inputs)   valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::DIGITAL_INPUTS);
    if (has_analog_inputs)    valid_fields_flags |= \
        static_cast<uint16_t>(ValidFieldFlag::ANALOG_INPUTS);

    // Assemble digital inputs
    uint16_t digital_inputs_flags = 0;
    for (size_t i = 0; i < 16; ++i) {
        if (this->digital_inputs[i]) {
            digital_inputs_flags |= (1 << i);
        }
    }

    // Serialize header
    if (!header.serialize(buffer, buffer_size)) {
        std::cerr << "RobotState::serialize: "
            << "Failed to serialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Serialize the rest of the data
    offset += serialize_type(sequence_delay, buffer + offset);
    offset += serialize_type(static_cast<uint8_t>(control_state), buffer + offset);
    offset += serialize_type(static_cast<uint8_t>(safety_status), buffer + offset);
    offset += serialize_type(static_cast<uint8_t>(operation_mode), buffer + offset);
    offset += serialize_type(static_cast<uint8_t>(operation_mode_status), buffer + offset);
    offset += serialize_type(status_flags, buffer + offset);
    offset += serialize_type(valid_fields_flags, buffer + offset);
    offset += serialize_array<double, 6>(joint_positions, buffer + offset);
    offset += serialize_array<double, 6>(joint_velocities, buffer + offset);
    offset += serialize_array<double, 6>(joint_torques, buffer + offset);
    offset += serialize_array<double, 6>(ft_sensor, buffer + offset);
    offset += serialize_type(digital_inputs_flags, buffer + offset);
    offset += serialize_array<double, 4>(analog_inputs, buffer + offset);

    if (offset != get_serialized_size()) {
        std::cerr << "RobotState::serialize: "
            << "Serialized size ( " << offset
            << " ) does not match expected size (" << get_serialized_size() << ")" << std::endl;
        return false;
    }

    return true;
}

bool RobotStateMessage::deserialize(uint8_t* buffer, size_t buffer_size)
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "RobotState::deserialize: "
            << "Buffer size is too small for RobotState" << std::endl;
        return false;
    }

    // Deserialize header
    if (!header.deserialize(buffer, buffer_size)) {
        std::cerr << "RobotState::deserialize: "
            << "Failed to deserialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Validate header fields

    // Deserialize the rest of the data
    offset += deserialize_type(buffer + offset, sequence_delay);
    uint8_t control_state_u8;
    offset += deserialize_type(buffer + offset, control_state_u8);
    control_state = static_cast<CommandType>(control_state_u8);
    uint8_t safety_status_u8;
    offset += deserialize_type(buffer + offset, safety_status_u8);
    safety_status = static_cast<SafetyStatus>(safety_status_u8);
    uint8_t operation_mode_u8;
    offset += deserialize_type(buffer + offset, operation_mode_u8);
    operation_mode = static_cast<OperationMode>(operation_mode_u8);
    uint8_t operation_mode_status_u8;
    offset += deserialize_type(buffer + offset, operation_mode_status_u8);
    operation_mode_status = operation_mode_status_u8;

    uint16_t status_flags;
    offset += deserialize_type(buffer + offset, status_flags);
    power_on         = (status_flags & static_cast<uint16_t>(StatusFlag::POWER_ON)) != 0;
    brakes_released     = (status_flags & static_cast<uint16_t>(StatusFlag::BRAKES_RELEASED)) != 0;
    motion_possible     = (status_flags & static_cast<uint16_t>(StatusFlag::MOTION_POSSIBLE)) != 0;
    in_motion           = (status_flags & static_cast<uint16_t>(StatusFlag::IN_MOTION)) != 0;
    error_state         = (status_flags & static_cast<uint16_t>(StatusFlag::ERROR_STATE)) != 0;
    wait_for_error_reset = \
        (status_flags & static_cast<uint16_t>(StatusFlag::WAIT_FOR_ERROR_RESET)) != 0;
    estop_pressed       = (status_flags & static_cast<uint16_t>(StatusFlag::ESTOP_PRESSED)) != 0;
    protective_stop     = (status_flags & static_cast<uint16_t>(StatusFlag::PROTECTIVE_STOP)) != 0;

    uint16_t valid_fields_flags;
    offset += deserialize_type(buffer + offset, valid_fields_flags);
    has_joint_positions  = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::JOINT_POSITIONS)) != 0;
    has_joint_velocities = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::JOINT_VELOCITIES)) != 0;
    has_joint_torques    = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::JOINT_TORQUES)) != 0;
    has_ft_sensor        = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::FT_SENSOR)) != 0;
    has_digital_inputs   = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::DIGITAL_INPUTS)) != 0;
    has_analog_inputs    = \
        (valid_fields_flags & static_cast<uint16_t>(ValidFieldFlag::ANALOG_INPUTS)) != 0;

    offset += deserialize_array<double, 6>(buffer + offset, joint_positions);
    offset += deserialize_array<double, 6>(buffer + offset, joint_velocities);
    offset += deserialize_array<double, 6>(buffer + offset, joint_torques);
    offset += deserialize_array<double, 6>(buffer + offset, ft_sensor);
    uint16_t digital_inputs_flags;
    offset += deserialize_type(buffer + offset, digital_inputs_flags);
    for (size_t i = 0; i < 16; ++i) {
        digital_inputs[i] = (digital_inputs_flags & (1 << i)) != 0;
    }
    offset += deserialize_array<double, 4>(buffer + offset, analog_inputs);

    if (offset != get_serialized_size()) {
        std::cerr << "RobotState::serialize: "
            << "Serialized size ( " << offset
            << " ) does not match expected size (" << get_serialized_size() << ")" << std::endl;
        return false;
    }

    return true;
}

// RobotCommand implementation

RobotCommandMessage::RobotCommandMessage() : Message()
{
    // Initialize header
    header.message_type = static_cast<uint8_t>(MessageType::ROBOT_COMMAND);
    header.payload_size = static_cast<uint16_t>(
        get_serialized_size() - FrameHeader::get_serialized_size());
    header.protocol_version = PROTOCOL_VERSION;
    header.magic_number = MAGIC_NUMBER;
    header.sequence_number = 0;
    // Initialize command
    command_type = CommandType::STOP;
    command_reference.fill(0.0);
    digital_outputs.fill(false);
    analog_outputs.fill(0.0);
}

size_t RobotCommandMessage::message_size() {
    // 8 (header) + 4 (controller period) + 1 (cmd type) + 6*4 (cmd data) + 2 (Doubt) + 4*4 (Aout)
    return 55;
}

bool RobotCommandMessage::serialize(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "RobotCommand::serialize: "
            << "Buffer size is too small for RobotCommand" << std::endl;
        return false;
    }

    // Validate header fields
    if (header.payload_size != get_serialized_size() - header.get_serialized_size()) {
        std::cerr << "RobotCommand::serialize: "
            << "Header payload size does not match RobotCommand size" << std::endl;
        return false;
    }
    if (header.message_type != static_cast<uint8_t>(MessageType::ROBOT_COMMAND)) {
        std::cerr << "RobotCommand::serialize: "
            << "Invalid message type: " << header.message_type << std::endl;
        return false;
    }

    // Serialize header
    if (!header.serialize(buffer, buffer_size)) {
        std::cerr << "RobotCommand::serialize: "
            << "Failed to serialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Serialize controller period
    offset += serialize_type(controller_period, buffer + offset);

    // Serialize motion command type
    offset += serialize_type(static_cast<uint8_t>(command_type), buffer + offset);
    offset += serialize_array<double, 6>(command_reference, buffer + offset);

    // Serialize digital outputs
    uint16_t digital_outputs_flags = 0;
    for (size_t i = 0; i < 16; ++i) {
        if (digital_outputs[i]) {
            digital_outputs_flags |= (1 << i);
        }
    }
    offset += serialize_type(digital_outputs_flags, buffer + offset);
    offset += serialize_array<double, 4>(analog_outputs, buffer + offset);

    if (offset != get_serialized_size()) {
        std::cerr << "RobotCommand::serialize: "
            << "Serialized size ( " << offset
            << " ) does not match expected size (" << get_serialized_size() << ")" << std::endl;
        return false;
    }

    return true;
}

bool RobotCommandMessage::deserialize(uint8_t* buffer, size_t buffer_size)
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "RobotCommand::deserialize: "
            << "Buffer size is too small for RobotCommand" << std::endl;
        return false;
    }

    // Deserialize header
    if (!header.deserialize(buffer, buffer_size)) {
        std::cerr << "RobotCommand::deserialize: "
            << "Failed to deserialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Validate header fields
    if (header.payload_size != get_serialized_size() - header.get_serialized_size()) {
        std::cerr << "RobotCommand::deserialize: "
            << "Header payload size does not match RobotCommand size" << std::endl;
        return false;
    }
    if (header.message_type != static_cast<uint8_t>(MessageType::ROBOT_COMMAND)) {
        std::cerr << "RobotCommand::deserialize: "
            << "Invalid message type: " << header.message_type << std::endl;
        return false;
    }

    // Deserialize controller period
    offset += deserialize_type(buffer + offset, controller_period);

    // Deserialize motion command
    uint8_t command_type_u8;
    offset += deserialize_type(buffer + offset, command_type_u8);
    command_type = static_cast<CommandType>(command_type_u8);
    offset += deserialize_array<double, 6>(buffer + offset, command_reference);

    // Deserialize digital outputs
    uint16_t digital_outputs_flags;
    offset += deserialize_type(buffer + offset, digital_outputs_flags);
    for (size_t i = 0; i < 16; ++i) {
        digital_outputs[i] = (digital_outputs_flags & (1 << i)) != 0;
    }
    offset += deserialize_array<double, 4>(buffer + offset, analog_outputs);

    if (offset != get_serialized_size()) {
        std::cerr << "RobotCommand::deserialize: "
            << "Serialized size ( " << offset
            << " ) does not match expected size (" << get_serialized_size() << ")" << std::endl;
        return false;
    }

    return true;
}

// DiagnosticData implementation

DiagnosticDataMessage::DiagnosticDataMessage() : Message()
{
    // Initialize header
    header.message_type = static_cast<uint8_t>(MessageType::DIAGNOSTIC_DATA);
    header.payload_size = static_cast<uint16_t>(
        get_serialized_size() - FrameHeader::get_serialized_size());
    header.protocol_version = PROTOCOL_VERSION;
    header.magic_number = MAGIC_NUMBER;
    header.sequence_number = 0;
    // Initialize diagnostic data
    robot_controller_type = RobotControllerType::UNKNOWN;
    robot_firmware_version = {0, 0, 0};
    for (auto& entry : error_log) {
        entry.error_code = 0;
        entry.timestamp = 0;
    }
    control_task_status = 0;
}

size_t DiagnosticDataMessage::message_size() { return 74; }

bool DiagnosticDataMessage::serialize(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "DiagnosticData::serialize: "
            << "Buffer size is too small for DiagnosticData" << std::endl;
        return false;
    }

    // Serialize header
    if (!header.serialize(buffer, buffer_size)) {
        std::cerr << "DiagnosticData::serialize: "
            << "Failed to serialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Validate header fields
    if (header.payload_size != get_serialized_size() - header.get_serialized_size()) {
        std::cerr << "DiagnosticData::serialize: "
            << "Header payload size does not match DiagnosticData size" << std::endl;
        return false;
    }
    if (header.message_type != static_cast<uint8_t>(MessageType::DIAGNOSTIC_DATA)) {
        std::cerr << "DiagnosticData::serialize: "
            << "Invalid message type: " << header.message_type << std::endl;
        return false;
    }

    // Serialize robot controller type
    offset += serialize_type(static_cast<uint8_t>(robot_controller_type), buffer + offset);

    // Serialize robot firmware version
    offset += serialize_type(robot_firmware_version.major, buffer + offset);
    offset += serialize_type(robot_firmware_version.minor, buffer + offset);
    offset += serialize_type(robot_firmware_version.patch, buffer + offset);

    // Serialize error log entries
    for (const auto& entry : error_log) {
        offset += serialize_type(entry.error_code, buffer + offset);
    }
    for (const auto& entry : error_log) {
        offset += serialize_type(entry.timestamp, buffer + offset);
    }

    // Serialize control task status
    offset += serialize_type(control_task_status, buffer + offset);

    // Validate serialized size
    if (offset != get_serialized_size()) {
        std::cerr << "DiagnosticData::serialize: "
            << "Serialized size does not match expected size" << std::endl;
        return false;
    }

    return true;
}

bool DiagnosticDataMessage::deserialize(uint8_t* buffer, size_t buffer_size)
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "DiagnosticData::deserialize: "
            << "Buffer size is too small for DiagnosticData" << std::endl;
        return false;
    }
    // Validate header fields
    if (header.payload_size != get_serialized_size() - header.get_serialized_size()) {
        std::cerr << "DiagnosticData::deserialize: "
            << "Header payload size does not match DiagnosticData size" << std::endl;
        return false;
    }
    if (header.message_type != static_cast<uint8_t>(MessageType::DIAGNOSTIC_DATA)) {
        std::cerr << "DiagnosticData::deserialize: "
            << "Invalid message type: " << header.message_type << std::endl;
        return false;
    }

    // Deserialize header
    if (!header.deserialize(buffer, buffer_size)) {
        std::cerr << "DiagnosticData::deserialize: "
            << "Failed to deserialize FrameHeader" << std::endl;
        return false;
    }
    size_t offset = header.get_serialized_size();

    // Deserialize robot controller type
    uint8_t controller_type_u8;
    offset += deserialize_type(buffer + offset, controller_type_u8);
    robot_controller_type = static_cast<RobotControllerType>(controller_type_u8);

    // Deserialize robot firmware version
    offset += deserialize_type(buffer + offset, robot_firmware_version.major);
    offset += deserialize_type(buffer + offset, robot_firmware_version.minor);
    offset += deserialize_type(buffer + offset, robot_firmware_version.patch);

    // Deserialize error log entries
    for (auto& entry : error_log) {
        offset += deserialize_type(buffer + offset, entry.error_code);
    }
    for (auto& entry : error_log) {
        offset += deserialize_type(buffer + offset, entry.timestamp);
    }

    // Deserialize control task status
    offset += deserialize_type(buffer + offset, control_task_status);

    if (offset != get_serialized_size()) {
        std::cerr << "DiagnosticData::deserialize: "
            << "Serialized size ( " << offset
            << " ) does not match expected size (" << get_serialized_size() << ")" << std::endl;
        return false;
    }

    return true;
}


}  // namespace staubli_robot_driver
