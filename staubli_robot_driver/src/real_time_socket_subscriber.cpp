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

#include "staubli_robot_driver/real_time_socket_subscriber.hpp"

#include <iostream>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace staubli_robot_driver {

// Constructor
template <typename MessageType>
RealTimeSocketSubscriber<MessageType>::RealTimeSocketSubscriber(std::shared_ptr<Socket> socket)
: socket_(socket),
  last_read_sequence_number_(0),
  logger_(rclcpp::get_logger("RealTimeSocketSubscriber")),
  first_read_(true)
{
    if (!init_communication()) {
        throw std::runtime_error("Failed to initialize communication");
    }
}

template <typename MessageType>
bool RealTimeSocketSubscriber<MessageType>::init_communication()
{
    RCLCPP_DEBUG(logger_, "Initializing reception thread...");
    if (!socket_) {
        RCLCPP_ERROR(
            logger_,
            "Cannot initialize communication: Invalid socket!");
        return false;
    }
    // Check socket connection
    if (!socket_->is_connected()) {
        RCLCPP_ERROR(
            logger_,
            "Cannot initialize communication: Socket is not connected");
        return false;
    }
    if (socket_->is_receiving()) {
        RCLCPP_ERROR(
            logger_,
            "Cannot initialize communication: Socket is already receiving");
        return false;
    }

    // Start the receive thread
    bool success = socket_->start_receive_thread(
        [this](std::vector<uint8_t>& data, size_t bytes_transferred) {
            // TODO(tpoignonec): change thread to real-time priority + CPU affinity
            update_message(data, bytes_transferred);
        }
    );
    if (success) {
        RCLCPP_INFO(logger_, "Communication initialized successfully");
    } else {
        RCLCPP_ERROR(logger_, "Cannot initialize communication: Failed to start receive thread");
    }
    return success;
}

// Destructor
template <typename MessageType>
RealTimeSocketSubscriber<MessageType>::~RealTimeSocketSubscriber() {
    socket_->stop_receive_thread();
}

// Update the message buffer with a new message
template <typename MessageType>
void RealTimeSocketSubscriber<MessageType>::update_message(
    std::vector<uint8_t>& data, size_t bytes_transferred)
{
    MessageType msg;
    if (msg.deserialize(data.data(), bytes_transferred)) {
        // Update the reception timestamp
        msg.reception_timestamp = rclcpp::Clock().now();
        // Acquire mutex and update the message buffer
        message_buffer_.set(msg);
    } else {
        RCLCPP_ERROR(
            rclcpp::get_logger("RealTimeSocketSubscriber"),
            "Failed to deserialize message of type %s",
            typeid(MessageType).name());
    }
}

// Read the latest message and check for lost packages and staleness
template <typename MessageType>
bool RealTimeSocketSubscriber<MessageType>::read_message(MessageType& msg, MessageStatus& status) {
    // Try to get the latest message from the buffer
    auto received_msg = message_buffer_.try_get();
    if (!received_msg.has_value()) {
        return false;
    }
    msg = received_msg.value();

    // Reset time since received (used to check staleness)
    status.time_since_received = rclcpp::Clock().now() - msg.reception_timestamp;

    // Assume we have a new message
    status.new_message = true;

    // Check for lost packages only after the first read
    if (!first_read_) {
        if (msg.header.sequence_number > last_read_sequence_number_) {
            // Normal case
            status.lost_packages = msg.header.sequence_number - last_read_sequence_number_ - 1;
        } else if (msg.header.sequence_number < last_read_sequence_number_) {
            // Handle wrap-around case
            status.lost_packages = (MAX_FRAME_SEQUENCE_NUMBER - last_read_sequence_number_) +
                msg.header.sequence_number + 1;
        } else {
            // Same sequence number, no lost packages
            status.lost_packages = 0;
            // This also means that we read the same message again
            status.new_message = false;
        }
    } else {
        status.lost_packages = 0;
        first_read_ = false;
    }

    // Compute expected next sequence number
    last_read_sequence_number_ = msg.header.sequence_number;

    return true;
}

template <typename MessageType>
bool RealTimeSocketSubscriber<MessageType>::is_ready() const {
    return socket_ && socket_->is_connected() && socket_->is_receiving();
}

}  // namespace staubli_robot_driver
