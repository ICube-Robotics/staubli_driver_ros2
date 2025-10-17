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

#ifndef STAUBLI_ROBOT_DRIVER__COMMUNICATION__TCP_SOCKET_HPP_
#define STAUBLI_ROBOT_DRIVER__COMMUNICATION__TCP_SOCKET_HPP_


#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "staubli_robot_driver/communication/socket.hpp"

namespace staubli_robot_driver {

// Forward declaration of implementation class
class TCPSocketImpl;

/**
 * @brief Class for handling bidirectional TCP communication
 */
class TCPSocket : public Socket {
public:
    TCPSocket();

    ~TCPSocket() override;

    bool connect(
        const std::string& remote_address,
        uint16_t remote_port,
        uint16_t local_port) override;

    bool connect(
        const std::string& remote_address,
        uint16_t remote_port,
        const std::string& local_address,
        uint16_t local_port) override;

    bool connect(
        const std::string& remote_address,
        uint16_t remote_port,
        uint16_t local_port,
        int timeout_ms);

    bool disconnect() override;

    bool is_connected() const override;

    bool send(std::vector<uint8_t>& data) override;

    bool receive_once(int timeout_ms, std::vector<uint8_t>& data) override;

    bool start_receive_thread(
        std::function<void(
            std::vector<uint8_t>& /*reception buffer*/,
            size_t /*bytes_transferred*/)> reception_callback) override;

    bool stop_receive_thread() override;

    bool is_receiving() const override;

private:
    std::unique_ptr<TCPSocketImpl> impl_;  // Implementation
};

/**
 * @brief Factory implementation for TCP communication
 */
class TCPSocketFactory {
public:
    /**
     * @brief Create a TCP communication interface
     * @return Shared pointer to TCP socket
     */
    static std::shared_ptr<Socket> create();
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__COMMUNICATION__TCP_SOCKET_HPP_
