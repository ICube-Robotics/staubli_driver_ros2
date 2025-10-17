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

#include "staubli_robot_driver/communication/tcp_socket.hpp"
#include "tcp_socket_impl.hpp"

#include <cstring>
#include <stdexcept>
#include <iostream>

namespace staubli_robot_driver {

TCPSocket::TCPSocket(): impl_(std::make_unique<TCPSocketImpl>()) { }

TCPSocket::~TCPSocket() { }

bool TCPSocket::connect(
    const std::string& remote_address, uint16_t remote_port, uint16_t local_port) {
    int default_timeout_ms = 2000;
    return this->connect(remote_address, remote_port, local_port, default_timeout_ms);
}

bool TCPSocket::connect(
    const std::string& remote_address,
    uint16_t remote_port,
    const std::string& local_address,
    uint16_t local_port) {
    // For TCP, the local address binding is typically not as commonly used as with UDP
    // This could be implemented later if needed for specific use cases
    // For now, delegate to the default implementation (bind to any interface)
    return connect(remote_address, remote_port, local_port);
}

bool TCPSocket::connect(
    const std::string& remote_address,
    uint16_t remote_port,
    uint16_t local_port,
    int timeout_ms) {
    return impl_->connect(remote_address, remote_port, local_port, timeout_ms);
}

bool TCPSocket::disconnect() {
    return impl_->disconnect();
}

bool TCPSocket::is_connected() const {
    return impl_->is_connected();
}

bool TCPSocket::send(std::vector<uint8_t>& data) {
    return impl_->send(data);
}

bool TCPSocket::receive_once(int timeout_ms, std::vector<uint8_t>& data) {
    return impl_->receive_once(timeout_ms, data);
}

bool TCPSocket::start_receive_thread(
    std::function<void(std::vector<uint8_t>&, size_t)> callback) {
    return impl_->start_receive_thread(callback);
}

bool TCPSocket::stop_receive_thread() {
    return impl_->stop_receive_thread();
}

bool TCPSocket::is_receiving() const {
    return impl_->is_receiving();
}

// TCPSocketFactory implementation

std::shared_ptr<Socket> TCPSocketFactory::create() {
    return std::make_shared<TCPSocket>();
}

}  // namespace staubli_robot_driver
