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

#ifndef STAUBLI_ROBOT_DRIVER__COMMUNICATION__PROTOCOL_HPP_
#define STAUBLI_ROBOT_DRIVER__COMMUNICATION__PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"

namespace staubli_robot_driver {

/**
 * @brief Magic number to identify the protocol
 */
constexpr uint16_t MAGIC_NUMBER = 0xABCD;

/**
 * @brief Protocol version
 */
constexpr uint8_t PROTOCOL_VERSION = 1;

/**
 * @brief Default port for the control socket
 */
constexpr uint16_t DEFAULT_CONTROL_PORT = 8080;

/**
 * @brief Default port for the diagnostics socket
 */
constexpr uint16_t DEFAULT_DIAGNOSTICS_PORT = 8081;

/**
 * @brief Maximal sequence number before wrap-around
 */
constexpr size_t MAX_FRAME_SEQUENCE_NUMBER = std::numeric_limits<uint16_t>::max();


/**
 * @brief Message types for communication
 */
enum class MessageType : uint8_t {
    INVALID = 0x00,
    ROBOT_STATE = 0x01,
    ROBOT_COMMAND = 0x02,
    DIAGNOSTIC_DATA = 0x03
};

/**
 * @brief Command types
 */
enum class CommandType : uint8_t {
    INVALID = 0x00,
    STOP = 0x01,
    JOINT_POSITION = 0x02,
    JOINT_VELOCITY = 0x03,
    JOINT_TORQUE = 0x04,
    // Used by VAL3 server, not valid command to send to robot
    DECELERATION_JOINT_POSITION = 0x0A,
    DECELERATION_JOINT_VELOCITY = 0x0B
};

/**
 * @brief Operation modes
 */
enum class OperationMode : uint8_t {
    UNKNOWN = 0x00,
    MANUAL = 0x01,
    AUTOMATIC = 0x02,
    REMOTE = 0x03
};

// /**
//  * @brief Operation mode statuses
//  */
// enum class OperationModeStatus : uint8_t {
//     UNKNOWN = 0x00,
//     PROGRAMMED_MOTION = 0x01,
//     CONNECTION_MOTION = 0x02,
//     HOLDING_POSITION = 0x03,
//     MANUAL_JOINT_JOG = 0x04,
//     MANUAL_FRAME_JOG = 0x05,
//     MANUAL_TOOL_JOG = 0x06,
//     MANUAL_POINT_JOG = 0x07
// };


/**
 * @brief Safety status
 */
enum class SafetyStatus : uint8_t {
    NO_SAFETY_STOP = 0x00,
    WAIT_FOR_RESTART = 0x01,
    SS1 = 0x02,
    SS2 = 0x03,
    WAIT_FOR_WMS = 0x04
};

/**
 * @brief Status flag (bit masks)
 */
enum class StatusFlag : uint16_t {
    POWER_ON = 0x0001,
    BRAKES_RELEASED = 0x0002,
    MOTION_POSSIBLE = 0x0004,
    IN_MOTION = 0x0008,
    ERROR_STATE = 0x0010,
    WAIT_FOR_ERROR_RESET = 0x0020,
    ESTOP_PRESSED = 0x0040,
    PROTECTIVE_STOP = 0x0080,
};

/**
 * @brief Valid fields (bit masks)
 */
enum class ValidFieldFlag : uint16_t {
    JOINT_POSITIONS = 0x0001,
    JOINT_VELOCITIES = 0x0002,
    JOINT_TORQUES = 0x0004,
    FT_SENSOR = 0x0008,
    DIGITAL_INPUTS = 0x0010,
    ANALOG_INPUTS = 0x0020,
};

/**
 * @brief Frame header structure
 */
struct FrameHeader {
    /// Magic number for identifying the protocol
    uint16_t magic_number = MAGIC_NUMBER;
    /// Protocol version (should be PROTOCOL_VERSION)
    uint8_t protocol_version = PROTOCOL_VERSION;
    /// Type of message
    uint8_t message_type = static_cast<uint8_t>(MessageType::INVALID);
    /// Sequence number for tracking messages
    uint16_t sequence_number = 0;
    /// Size of the payload in bytes
    uint16_t payload_size = 0;

    /// Serialize header to bytes
    bool serialize(uint8_t* buffer, size_t buffer_size) const;

    /// Deserialize bytes to header
    bool deserialize(uint8_t* buffer, size_t buffer_size);

    /// Size of the header in bytes
    static size_t get_serialized_size() { return 8; }
};

/**
 * @brief Generic message structure
 *
 * @details This is a base class for all messages in the protocol.
 *
 * Message format:
 *   ______________________________
 *  |            |                 |
 *  |   Header   |     Payload     |
 *  | (8 bytes)  | (variable size) |
 *  |____________|_________________|
 *
 * Header format (8 bytes):
 *  - Magic number (2 bytes, uint16_t)
 *  - Protocol version (1 bytes, uint8_t)
 *  - Message type (1 bytes, uint8_t)
 *      * ROBOT_STATE = 0x01
 *      * ROBOT_COMMAND = 0x02
 *      * DIAGNOSTIC_DATA = 0x03
 *  - Sequence number (2 bytes, uint16_t)
 *  - Payload size (2 bytes, uint16_t)
 */
class Message {
public:
    Message() = default;
    virtual ~Message() = default;

    /// Size of the entire message in bytes
    virtual size_t get_serialized_size() const = 0;

    /// Serialize message to bytes
    virtual bool serialize(uint8_t* buffer, size_t buffer_size) const = 0;

    /// Deserialize bytes to message
    virtual bool deserialize(uint8_t* buffer, size_t buffer_size) = 0;

    // Convenience methods for std::vector

    /**
     * @brief Serialize message to vector
     *
     * @warning This method modifies the input vector by resizing it to the
     *          size of the message.
     *
     * @param data Output data buffer. Capacity must be at least `message.get_serialized_size()`.
     * @return true if serialization was successful, false otherwise
     */
    bool serialize(std::vector<uint8_t>& data) const;

    /**
     * @brief Deserialize message from vector
     *
     * @warning The input vector must be at least the size of the message.
     *
     * @param data Input data buffer
     * @return true if deserialization was successful, false otherwise
     */
    bool deserialize(std::vector<uint8_t>& data);

public:
    /// Message header
    FrameHeader header;

    /// Timestamp of the message reception (local clock)
    rclcpp::Time reception_timestamp = rclcpp::Time(0);
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__COMMUNICATION__PROTOCOL_HPP_
