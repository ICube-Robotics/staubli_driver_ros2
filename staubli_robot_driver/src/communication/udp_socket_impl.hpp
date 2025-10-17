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


#ifndef COMMUNICATION__UDP_SOCKET_IMPL_HPP_
#define COMMUNICATION__UDP_SOCKET_IMPL_HPP_

#include "staubli_robot_driver/communication/udp_socket.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#include <rclcpp/logging.hpp>

namespace staubli_robot_driver {

/**
 * @brief Class representing a UDP socket for communication using Asio
 */
class UDPSocketImpl {
public:
    UDPSocketImpl();
    ~UDPSocketImpl();

    bool connect(const std::string& remote_address, uint16_t remote_port,
                 const std::string& local_address, uint16_t local_port);
    bool disconnect();
    bool is_connected() const;
    bool send(std::vector<uint8_t>& data);
    bool receive_once(int timeout_ms, std::vector<uint8_t>& data);
    bool start_receive_thread(
        std::function<void(
            std::vector<uint8_t>& /*reception buffer*/,
            size_t /*bytes_transferred*/)> reception_callback);
    bool stop_receive_thread();
    bool is_receiving() const;

protected:
    void start_receive();
    void handle_data_received(
        const boost::system::error_code& error,
        std::size_t bytes_transferred);

    rclcpp::Logger logger_;

private:
    // Flag indicating if socket is connected
    std::atomic<bool> is_connected_{false};
    std::atomic<bool> receiving_{false};  // Flag indicating if the receive thread is running

    // Asio components
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint local_endpoint_, remote_endpoint_;

    // Buffer for receiving data
    std::vector<uint8_t> recv_buffer_;

    // Receive thread
    std::thread receive_thread_;
    std::function<void(std::vector<uint8_t>&, size_t)> reception_callback_;
};

// UDPSocketImpl implementation

UDPSocketImpl::UDPSocketImpl()
: logger_(rclcpp::get_logger("staubli_robot_driver::UDPSocket")),
  is_connected_(false),
  receiving_(false),
  socket_(io_service_)
{
    recv_buffer_.resize(MAX_SOCKET_PACKET_SIZE);  // Max UDP packet size
}

UDPSocketImpl::~UDPSocketImpl() {
    if (is_connected()) {
        disconnect();
    }
}

bool UDPSocketImpl::connect(
    const std::string& remote_address,
    uint16_t remote_port,
    const std::string& local_address,
    uint16_t local_port)
{
    RCLCPP_INFO(logger_, "Connecting UDP socket to %s:%d from %s:%d...",
        remote_address.c_str(), remote_port, local_address.c_str(), local_port);
    // If already connected, disconnect first
    if (is_connected()) {
        RCLCPP_WARN(logger_, "UDP socket is already connected. Disconnecting...");
        if (disconnect() == false) {
            return false;
        }
    }

    try {
        // Open the socket
        socket_.open(boost::asio::ip::udp::v4());

        // Set socket options for better reusability
        socket_.set_option(boost::asio::socket_base::reuse_address(true));

        // Create local endpoint with specified IP address
        boost::asio::ip::address local_addr;
        if (local_address == "0.0.0.0" || local_address.empty()) {
            // Bind to all interfaces
            local_addr = boost::asio::ip::address_v4::any();
        } else {
            // Bind to specific interface
            local_addr = boost::asio::ip::address::from_string(local_address);
        }

        local_endpoint_ = boost::asio::ip::udp::endpoint(local_addr, local_port);
        socket_.bind(local_endpoint_);

        // Update local_endpoint_ with the actual bound port
        local_endpoint_ = socket_.local_endpoint();

        RCLCPP_DEBUG(logger_, "UDP socket bound to local address %s:%d",
                   local_endpoint_.address().to_string().c_str(), local_endpoint_.port());

        // Resolve remote endpoint
        boost::asio::ip::udp::resolver resolver(io_service_);
        boost::asio::ip::udp::resolver::query query(
            boost::asio::ip::udp::v4(), remote_address, std::to_string(remote_port));
        remote_endpoint_ = *resolver.resolve(query);
        RCLCPP_DEBUG(logger_, "UDP socket will send to %s:%d",
                   remote_address.c_str(), remote_port);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error connecting UDP socket: %s", e.what());
        if (socket_.is_open()) {
            socket_.close();
        }
        is_connected_.store(false);
        return false;
    }

    is_connected_.store(true);
    RCLCPP_DEBUG(logger_, "UDP socket connected");
    return true;
}

bool UDPSocketImpl::disconnect() {
    RCLCPP_DEBUG(logger_, "Disconnecting UDP socket...");
    bool all_ok = true;
    if (is_connected()) {
        if (is_receiving()) {
            RCLCPP_WARN(logger_,
                "Trying to disconnect the socket while the receive "
                "thread is running. Stopping it first..."
            );
            all_ok &= stop_receive_thread();
        }
        try {
            socket_.close();
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error disconnecting the UDP socket: %s", e.what());
            all_ok = false;
        }
        is_connected_.store(false);
        RCLCPP_DEBUG(logger_, "UDP socket disconnected");
    } else {
        RCLCPP_WARN(logger_, "Cannot disconnect UDP socket: not connected");
    }
    return all_ok;
}

bool UDPSocketImpl::is_connected() const {
    return is_connected_.load();
}

bool UDPSocketImpl::send(std::vector<uint8_t>& data) {
    if (!is_connected()) {
        RCLCPP_WARN(logger_, "Error sending UDP packet: UDP socket is not connected");
        return false;
    }
    boost::system::error_code err;

    // Send data
    RCLCPP_DEBUG(logger_, "Sending %zu bytes via UDP", data.size());
    socket_.send_to(
        boost::asio::buffer(data.data(), data.size()),
        remote_endpoint_, 0, err);

    if (err.value() != 0) {
        RCLCPP_ERROR(logger_, "Error sending UDP packet: %s", err.message().c_str());
        return false;
    }

    return true;
}

bool UDPSocketImpl::receive_once(int timeout_ms, std::vector<uint8_t>& data)
{
    if (!is_connected()) {
        RCLCPP_WARN(logger_, "Cannot receive data: UDP socket is not connected");
        return false;
    }
    if (is_receiving()) {
        RCLCPP_WARN(logger_, "Cannot perform blocking receive: receive thread is running");
        return false;
    }
    std::promise<bool> callback_promise;
    std::future<bool> callback_future = callback_promise.get_future();
    auto callback = [&data, &callback_promise]
                    (std::vector<uint8_t>& data_read, size_t bytes_transferred) {
        data.resize(bytes_transferred);
        std::copy(data_read.begin(), data_read.begin() + bytes_transferred, data.begin());
        callback_promise.set_value(true);
    };

    // Start receive thread with the callback
    if (!start_receive_thread(callback)) {
        RCLCPP_ERROR(logger_, "Failed to start receive thread for blocking receive");
        return false;
    }

    // Wait for data or timeout
    bool received = true;
    auto status = callback_future.wait_for(std::chrono::milliseconds(timeout_ms));
    if (status == std::future_status::timeout) {
        RCLCPP_WARN(logger_, "Timeout waiting for UDP data");
        received = false;
    }

    // Stop receive thread
    if (!stop_receive_thread()) {
        RCLCPP_ERROR(logger_, "Failed to stop receive thread after receive_once");
    }

    return received;
}

bool UDPSocketImpl::start_receive_thread(
    std::function<void(std::vector<uint8_t>&, size_t)> reception_callback)
{
    RCLCPP_DEBUG(logger_, "Starting UDP receive thread...");
    // Set the callback
    reception_callback_ = reception_callback;
    // Only start if the socket is open and connected
    if (!is_connected()) {
        RCLCPP_WARN(logger_, "Cannot start receive thread: socket is not connected.");
        return false;
    }
    if (is_receiving()) {
        RCLCPP_WARN(logger_, "Receive thread is already running.");
        return false;
    }

    if (recv_buffer_.size() != MAX_SOCKET_PACKET_SIZE) {
        recv_buffer_.resize(MAX_SOCKET_PACKET_SIZE);  // Max UDP packet size
    }

    // Setup
    io_service_.reset();
    this->start_receive();

    // Start the receive thread
    receiving_.store(true);
    receive_thread_ = std::thread([this]() {
        // TODO(tpoignonec): elevate priority of this thread?
        io_service_.run();
        RCLCPP_DEBUG(logger_, "ASIO service stopped");
    });
    RCLCPP_DEBUG(logger_, "UDP receive thread started");
    return true;
}

bool UDPSocketImpl::stop_receive_thread() {
    RCLCPP_DEBUG(logger_, "Stopping UDP receive thread...");
    if (!is_receiving()) {
        RCLCPP_WARN(logger_, "Cannot stop receive thread: not running.");
        return true;  // Already stopped
    }
    io_service_.stop();
    receive_thread_.join();
    receiving_.store(false);
    RCLCPP_DEBUG(logger_, "UDP receive thread stopped");
    return true;
}

bool UDPSocketImpl::is_receiving() const {
    return receiving_.load();
}

void UDPSocketImpl::handle_data_received(
    const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
    RCLCPP_DEBUG(logger_, "UDP packet received: %zu bytes", bytes_transferred);
    if (error) {
        RCLCPP_ERROR(logger_, "Error in UDP receive: %s", error.message().c_str());
    }
    // Call the user callback
    RCLCPP_DEBUG(logger_, "Invoking reception callback");
    this->reception_callback_(recv_buffer_, bytes_transferred);
    // Wait for the next reception
    if (is_receiving()) {
        this->start_receive();
    }
    return;
}

void UDPSocketImpl::start_receive() {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_, recv_buffer_.size()),
        remote_endpoint_,
        [this](auto error, auto bytes_transferred) {
            // Handle the event
            handle_data_received(error, bytes_transferred);
        }
    );
}

}  // namespace staubli_robot_driver

#endif  // COMMUNICATION__UDP_SOCKET_IMPL_HPP_
