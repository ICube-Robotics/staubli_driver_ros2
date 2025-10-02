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

#ifndef STAUBLI_ROBOT_DRIVER__COMMUNICATION__MESSAGES_HPP_
#define STAUBLI_ROBOT_DRIVER__COMMUNICATION__MESSAGES_HPP_

#include "staubli_robot_driver/communication/protocol.hpp"


namespace staubli_robot_driver {

/**
 * @brief Robot state message
 *
 * @details This message contains the current state of the robot.
 *          It is sent periodically by the robot controller.
 *
 * Payload format:
 *   - Operation mode (1 byte, uint8_t)
 *   - Robot status flag array (2 bytes, uint16_t), see `StatusFlag` enum
 *   - Valid fields flag array (2 bytes, uint16_t), see `ValidFieldFlag` enum
 *   - Joint positions (6 * 4 bytes, float array)
 *   - Joint velocities (6 * 4 bytes, float array)
 *   - Joint torques (6 * 4 bytes, float array)
 *   - F/T sensor readings (6 * 4 bytes, float array)
 *   - Digital inputs (2 bytes, 16 bits)
 *      D_in_0 = bit 0 (LSB), D_in_15 = bit 15 (MSB)
 *      In the frame: [D_in_15, D_in_14, ..., D_in_0], over two bytes
 *   - Analog inputs (4 * 4 bytes, float array)
 *      Each element represents the value of an analog input.
 *      In the frame: [A_in_1], [A_in_2], [A_in_3], [A_in_4]
 *
 * @note Array sizes are fixed for Staubli robots with 6 axes.
 *
 * @note Arrays are serialized to the frame in order. For example, joint positions
 *       are serialized as [J1, J2, J3, J4, J5, J6]. The only exception is the
 *       digital inputs, which are serialized as a 16-bit integer with each bit
 *
 * @note Joint positions, velocities, torque, and F/T sensor readings are
 *       only included if the corresponding valid field flag is set. Otherwise,
 *       they are filled with zeros. The same applies to digital and analog
 *       inputs. However, if the robot does not have these inputs, they are also
 *       filled with zeros.
 */
class RobotStateMessage: public Message {
public:
    RobotStateMessage();
    ~RobotStateMessage() = default;

    // Bring the vector-based methods from the base class into scope
    using Message::serialize;
    using Message::deserialize;

    /// Serialize message to bytes
    bool serialize(uint8_t* buffer, size_t buffer_size) const override;

    /// Deserialize bytes to message
    bool deserialize(uint8_t* buffer, size_t buffer_size) override;

    /// Get the size of the message, static version
    static size_t message_size();

    /// Size of the entire message in bytes
    size_t get_serialized_size() const override {
        return message_size();
    }

public:
    // Robot operation mode
    OperationMode operation_mode = OperationMode::UNKNOWN;

    // Robot status flags
    bool power_on = false;
    bool brakes_released = false;
    bool motion_possible = false;
    bool in_motion = false;
    bool error_state = false;
    bool wait_for_error_reset = false;
    bool estop_pressed = false;
    bool protective_stop = false;

    // Valid fields flags
    bool has_joint_positions = false;
    bool has_joint_velocities = false;
    bool has_joint_torques = false;
    bool has_ft_sensor = false;
    bool has_digital_inputs = false;
    bool has_analog_inputs = false;

    // Joint positions in radians
    std::array<double, 6> joint_positions;

    // Joint velocities in radians/s
    std::array<double, 6> joint_velocities;

    // Joint torques in Nm
    std::array<double, 6> joint_torques;

    // F/T sensor readings in N and Nm
    std::array<double, 6> ft_sensor;

    // Digital inputs
    std::array<bool, 16> digital_inputs;

    // Analog inputs
    std::array<double, 4> analog_inputs;
};

/**
 * @brief Robot command message
 */
class RobotCommandMessage : public Message {
public:
    RobotCommandMessage();
    ~RobotCommandMessage() = default;

    // Bring the vector-based methods from the base class into scope
    using Message::serialize;
    using Message::deserialize;

    /// Serialize message to bytes
    bool serialize(uint8_t* buffer, size_t buffer_size) const override;

    /// Deserialize bytes to message
    bool deserialize(uint8_t* buffer, size_t buffer_size) override;

    /// Get the size of the message, static version
    static size_t message_size();

    /// Size of the entire message in bytes
    size_t get_serialized_size() const override {
        return message_size();
    }

public:
    // Command type (position, velocity, torque)
    CommandType command_type = CommandType::STOP;

    /**
     * @brief Command reference for the robot
     *
     * @details This array contains the target values for the robot's joints.
     *          The interpretation of these values depends on the command type:
     *             - JOINT_POSITION: Target joint positions in radians
     *             - JOINT_VELOCITY: Target joint velocities in radians/s
     *             - JOINT_TORQUE: Target joint torques in Nm
     *             - STOP: Ignored, should be filled with zeros
     */
    std::array<double, 6> command_reference;

    // Digital outputs
    std::array<bool, 16> digital_outputs;

    // Analog outputs
    std::array<double, 4> analog_outputs;
};

/**
 * @brief Diagnostic data
 */
class DiagnosticDataMessage : public Message {
public:
    DiagnosticDataMessage();
    ~DiagnosticDataMessage() = default;

    // Bring the vector-based methods from the base class into scope
    using Message::serialize;
    using Message::deserialize;

    /// Serialize message to bytes
    bool serialize(uint8_t* buffer, size_t buffer_size) const override;

    /// Deserialize bytes to message
    bool deserialize(uint8_t* buffer, size_t buffer_size) override;

    /// Get the size of the message, static version
    static size_t message_size();

    /// Size of the entire message in bytes
    size_t get_serialized_size() const override {
        return message_size();
    }

public:
    // Robot controller type
    enum class RobotControllerType : uint8_t {
        UNKNOWN = 0x00,
        CS8 = 0x01,
        CS9 = 0x02,
    };
    RobotControllerType robot_controller_type;

    // Robot firmware major version
    struct Version {
        uint8_t major;
        uint8_t minor;
        uint8_t patch;
    };
    Version robot_firmware_version;

    // Error log entries, maximum 10 entries
    struct ErrorLogEntry {
        uint32_t timestamp;  ///< Timestamp of the error in milliseconds
        uint16_t error_code;  ///< Error code, 0 if no error
    };
    std::array<ErrorLogEntry, 10> error_log;

    // Robot task statuses
    uint16_t control_task_status;
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__COMMUNICATION__MESSAGES_HPP_
