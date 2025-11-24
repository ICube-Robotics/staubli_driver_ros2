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

#ifndef STAUBLI_ROBOT_DRIVER__TYPES_HPP_
#define STAUBLI_ROBOT_DRIVER__TYPES_HPP_

namespace staubli_robot_driver {

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
    ESTOP_PRESSED = 0x0020,
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

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__TYPES_HPP_
