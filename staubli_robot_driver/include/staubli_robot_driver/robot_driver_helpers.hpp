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


#ifndef STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HELPERS_HPP_
#define STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HELPERS_HPP_

// C++
#include <algorithm>
#include <vector>

// Staubli Robot Driver
#include "staubli_robot_driver/robot_driver.hpp"

namespace staubli_robot_driver {

bool set_command_stop(RobotCommandMessage& msg)
{
    msg.command_type = CommandType::STOP;
    msg.command_reference.fill(0.0);
    return true;
}

bool set_command_joint_position(RobotCommandMessage& msg, std::vector<double> joint_positions)
{
    if (joint_positions.size() != msg.command_reference.size()) {
        return false;
    }
    msg.command_type = CommandType::JOINT_POSITION;
    std::copy(joint_positions.begin(), joint_positions.end(), msg.command_reference.begin());
    return true;
}

bool set_command_joint_velocity(RobotCommandMessage& msg, std::vector<double> joint_velocities)
{
    if (joint_velocities.size() != msg.command_reference.size()) {
        return false;
    }
    msg.command_type = CommandType::JOINT_VELOCITY;
    std::copy(joint_velocities.begin(), joint_velocities.end(), msg.command_reference.begin());
    return true;
}


bool set_sequence_number(RobotCommandMessage& msg, RobotStateMessage& state)
{
    msg.header.sequence_number = state.header.sequence_number;
    return true;
}

/**
 * @brief Clear and prepare a RobotCommandMessage
 *
 * @param msg Message to clear
 * @param state Last read robot state (to set sequence number)
 */
bool prepare_robot_command_message(RobotCommandMessage& msg, RobotStateMessage& state)
{
    bool result = true;
    result &= set_sequence_number(msg, state);
    // Set command
    result &= set_command_stop(msg);
    // Command robot stop and clear IOs
    msg.digital_outputs.fill(false);
    msg.analog_outputs.fill(0.0);
    return result;
}

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__ROBOT_DRIVER_HELPERS_HPP_
