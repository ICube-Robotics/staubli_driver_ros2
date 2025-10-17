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

#include "staubli_robot_driver/communication/udp_socket.hpp"
#include "udp_socket_impl.hpp"

#include <cstring>
#include <stdexcept>
#include <iostream>

namespace staubli_robot_driver {

UDPSocket::UDPSocket(): impl_(std::make_unique<UDPSocketImpl>()) { }

UDPSocket::~UDPSocket() { }

bool UDPSocket::connect(
    const std::string& remote_address, uint16_t remote_port, uint16_t local_port) {
    return impl_->connect(remote_address, remote_port, "", local_port);
}

bool UDPSocket::connect(
    const std::string& remote_address, uint16_t remote_port,
    const std::string& local_address, uint16_t local_port) {
    return impl_->connect(remote_address, remote_port, local_address, local_port);
}

bool UDPSocket::disconnect() {
    return impl_->disconnect();
}

bool UDPSocket::is_connected() const {
    return impl_->is_connected();
}

bool UDPSocket::send(std::vector<uint8_t>& data) {
    return impl_->send(data);
}

bool UDPSocket::receive_once(int timeout_ms, std::vector<uint8_t>& data) {
    return impl_->receive_once(timeout_ms, data);
}

bool UDPSocket::start_receive_thread(
    std::function<void(std::vector<uint8_t>&, size_t)> callback) {
    return impl_->start_receive_thread(callback);
}

bool UDPSocket::stop_receive_thread() {
    return impl_->stop_receive_thread();
}

bool UDPSocket::is_receiving() const {
    return impl_->is_receiving();
}

// UDPSocketFactory implementation

std::shared_ptr<Socket> UDPSocketFactory::create() {
    return std::make_shared<UDPSocket>();
}

}  // namespace staubli_robot_driver
