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

#ifndef STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_INTERFACE_HPP_
#define STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_INTERFACE_HPP_

#include <cstdint>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "staubli_robot_driver/real_time_socket_subscriber.hpp"

namespace staubli_robot_driver {

/**
 * @brief Bilateral real-time socket interface for pub/sub communication
 *
 * @tparam MessageSub Type of message to subscribe to
 * @tparam MessagePub Type of message to publish
 */
template <typename MessageSub, typename MessagePub>
class RealTimeSocketInterface : public RealTimeSocketSubscriber<MessageSub> {
public:
    explicit RealTimeSocketInterface(std::shared_ptr<Socket> socket);
    ~RealTimeSocketInterface();

    /**
     * @brief Serialize and send a message over the socket.
     *
     * @warning This is a blocking call.
     *
     * @param msg Message to send
     * @return true if the message was sent successfully, false otherwise
     */
    bool send_message(const MessagePub& msg);

    /**
     * @brief Check if the interface is ready (socket connected and receiving)
     * @return true if the interface is ready, false otherwise
     */
    bool is_ready() const override {
        return RealTimeSocketSubscriber<MessageSub>::is_ready();
    }

private:
    std::vector<uint8_t> data_out_;  // Buffer for serialized message
};

// Declarations for message types
template class RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>;
template class RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>;

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__REAL_TIME_SOCKET_INTERFACE_HPP_
