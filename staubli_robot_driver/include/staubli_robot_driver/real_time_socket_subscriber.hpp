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

#ifndef STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_SUBSCRIBER_HPP_
#define STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_SUBSCRIBER_HPP_

// C++
#include <atomic>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

// Realtime tools (ROS2 package)
#include <realtime_tools/realtime_thread_safe_box.hpp>

// Staubli Robot Driver
#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/communication/protocol.hpp"
#include "staubli_robot_driver/communication/socket.hpp"

namespace staubli_robot_driver {

/// Status struct to hold message status information
struct MessageStatus {
    /// True if a new message was received
    bool new_message;
    /// Time since the message was received (see `msg.reception_timestamp`)
    rclcpp::Duration time_since_received{1, 0};
    /// Number of lost packages (0 if none)
    size_t lost_packages;
};

/**
 * @brief Real-time socket subscriber class template
 *
 * @details Subscribes to messages of type `MessageType` over a socket.
 *          Handles message reception and status tracking in a thread-safe manner.
 *
 * @tparam MessageType Type of message to subscribe to
 */
template <typename MessageType>
class RealTimeSocketSubscriber {
public:
    explicit RealTimeSocketSubscriber(std::shared_ptr<Socket> socket);
    virtual ~RealTimeSocketSubscriber();

    // Read the latest message and check for lost packages and staleness

    /**
     * @brief Read the latest message and check for lost packages and staleness
     *
     * @details This method retrieves the most recent message received over the socket,
     *          along with its status information. It checks for lost packages based on
     *          sequence numbers and computes the time since the message was received
     *          (time of deserialization in async thread).
     *
     *          This method is thread-safe and can be called from real-time contexts.
     *          If no message AT ALL has been received yet, it returns `false`.
     *          If no message was received since last call, but one had arrived at
     *          some point before, the method returns `true`, but with the `status.new_message`
     *          field set to `false`.
     *
     *          In any case, the staleness of the message should be checked using
     *          the `status.time_since_received` field.
     *
     * @param msg Reference to store the received message
     * @param status Reference to store the message status information
     * @return true if a message was received (at some point) and that the returned message
     *          is valid, false otherwise. Check `status` in any case...
     */
    bool read_message(MessageType& msg, MessageStatus& status);

    /**
     * @brief Check if the subscriber is ready (socket connected and receiving)
     * @return true if the subscriber is ready, false otherwise
     */
    virtual bool is_ready() const;

protected:
    // Communication initialization
    bool init_communication();

    // Update the message buffer with a new message
    void update_message(std::vector<uint8_t>& data, size_t bytes_transferred);

    // Error handling
    void handle_error(const std::string& error_msg);

protected:
    std::shared_ptr<Socket> socket_;
    /// Sequence number of the last read RobotStateMessage
    size_t last_read_sequence_number_;

    rclcpp::Logger logger_;

private:
    realtime_tools::RealtimeThreadSafeBox<MessageType> message_buffer_;
    bool first_read_;  // Flag to indicate if it's the first read
};

// Declarations for message types
template class RealTimeSocketSubscriber<DiagnosticDataMessage>;
template class RealTimeSocketSubscriber<RobotCommandMessage>;
template class RealTimeSocketSubscriber<RobotStateMessage>;

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_SUBSCRIBER_HPP_
