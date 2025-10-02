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

#ifndef STAUBLI_ROBOT_DRIVER__COMMUNICATION__SOCKET_HPP_
#define STAUBLI_ROBOT_DRIVER__COMMUNICATION__SOCKET_HPP_

#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace staubli_robot_driver {

/**
 * @brief Maximum packet size (bytes) for socket communication
 */
constexpr size_t MAX_SOCKET_PACKET_SIZE = 1024;

/**
 * @brief Abstract interface for communication with the robot
 *
 * This interface defines the methods required for any communication
 * protocol implementation (UDP / TCP)
 */
class Socket {
public:
    /**
     * @brief Default constructor
     */
    Socket() = default;

    /**
     * @brief Virtual destructor
     */
    virtual ~Socket() = default;

    /**
     * @brief Connect to the remote endpoint
     * @param remote_address Address of the remote endpoint (format depends on protocol)
     * @param remote_port Port or identifier for the remote endpoint
     * @param local_port Port or identifier for the local endpoint
     * @return True if connected successfully, false otherwise
     */
    virtual bool connect(
        const std::string& remote_address, uint16_t remote_port, uint16_t local_port) = 0;

    /**
     * @brief Disconnect from the remote endpoint
     * @return True if disconnected successfully, false otherwise
     */
    virtual bool disconnect() = 0;

    /**
     * @brief Check if the connection is established
     * @return True if connected, false otherwise
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Send data to the remote endpoint
     * @param data Data to send
     * @return True if data was sent successfully, false otherwise
     */
    virtual bool send(std::vector<uint8_t>& data) = 0;

    /**
     * @brief Receive data from socket (blocking with timeout)
     * @param timeout_ms Timeout in milliseconds
     * @param data Will be filled with received data
     * @return Number of bytes received, 0 on timeout, -1 on error
     *
     * @warning The data vector must be preallocated with the max buffer size.
     *       The function will resize it to the max buffer size otherwise...
     */
    virtual bool receive_once(int timeout_ms, std::vector<uint8_t>& data) = 0;

    /**
     * @brief Start a receive thread that calls the provided reception_callback when data is received
     * @param reception_callback Function to call when data is received
     * @return True if the receive thread was started successfully, false otherwise
     */
    virtual bool start_receive_thread(
        std::function<void(
            std::vector<uint8_t>& /*reception buffer*/,
            size_t /*bytes_transferred*/)> reception_callback) = 0;

    /**
     * @brief Stop the receive thread
     * @return True if the receive thread was stopped successfully, false otherwise
     */
    virtual bool stop_receive_thread() = 0;

    /**
     * @brief Check if the receive thread is running
     * @return True if the receive thread is running, false otherwise
     */
    virtual bool is_receiving() const = 0;
};

/**
 * @brief Factory for creating communication interfaces
 */
class SocketFactory {
public:
    /**
     * @brief Communication protocol types
     */
    enum class ProtocolType {
        UDP,   ///< UDP socket communication
        TCP,   ///< TCP socket communication
    };

    /**
     * @brief Create a communication interface
     * @param type Type of protocol to use
     * @return Shared pointer to the created communication interface
     */
    static std::shared_ptr<Socket> create(ProtocolType type);
};

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__COMMUNICATION__SOCKET_HPP_
