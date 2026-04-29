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

#include "staubli_robot_driver/real_time_socket_interface.hpp"

#include <iostream>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace staubli_robot_driver {


template <typename MessageSub, typename MessagePub>
RealTimeSocketInterface<MessageSub, MessagePub>::RealTimeSocketInterface(
    std::shared_ptr<Socket> socket) : RealTimeSocketSubscriber<MessageSub>(socket)
{
    // Rename the logger
    this->logger_ = rclcpp::get_logger("RealTimeSocketInterface");
    // Preallocate buffer for outgoing messages
    this->data_out_.resize(MessagePub::message_size(), 0);
}

template <typename MessageSub, typename MessagePub>
RealTimeSocketInterface<MessageSub, MessagePub>::~RealTimeSocketInterface() {
    // Nothing to do here, see parent class
}

template <typename MessageSub, typename MessagePub>
bool RealTimeSocketInterface<MessageSub, MessagePub>::send_message(const MessagePub& msg) {
    // Check socket connection
    if (!this->socket_->is_connected()) {
        RCLCPP_ERROR(this->logger_, "Cannot send message: socket is not connected");
        return false;
    }
    // Serialize the message
    if (!msg.serialize(data_out_.data(), data_out_.size())) {
        RCLCPP_ERROR(this->logger_, "Cannot send message: serialization failed");
        return false;
    }
    // Send the message
    if (!this->socket_->send(data_out_)) {
        RCLCPP_ERROR(this->logger_, "Cannot send message: send over socket failed");
        return false;
    }
    return true;
}

}  // namespace staubli_robot_driver
