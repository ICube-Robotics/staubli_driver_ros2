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

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "staubli_robot_driver/communication/socket.hpp"

// Global flag for clean shutdown
std::atomic<bool> running{true};

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ". Shutting down..." << std::endl;
    running = false;
}

int main() {
    rclcpp::init(0, nullptr);

    // Install signal handler for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "Starting simple message receiver..." << std::endl;

    // Create UDP socket
    auto socket = staubli_robot_driver::SocketFactory::create(
        staubli_robot_driver::SocketFactory::ProtocolType::UDP);

    if (!socket) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    // Connect to robot (robot will send messages to us)
    std::string robot_ip = "192.168.0.254";  // Default robot IP
    std::string local_ip = "";  // Local IP address to bind to ("" means any)
    uint16_t local_port = 11000;  // Port we listen on
    uint16_t robot_port = 11000;  // Port robot sends from

    std::cout << "Connecting to robot at " << robot_ip << ":" << robot_port
              << " (listening on " << local_ip << ":" << local_port << ")" << std::endl;

    if (!socket->connect(robot_ip, robot_port, local_ip, local_port)) {
        std::cerr << "Failed to connect socket" << std::endl;
        return -1;
    }

    std::cout << "Connected! Waiting for messages..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    // Message counter
    size_t message_count = 0;

    while (running) {
        // Buffer for receiving data
        std::vector<uint8_t> received_data;

        // Try to receive data with timeout (1000ms)
        if (socket->receive_once(1000, received_data)) {
            message_count++;
            std::cout << "Received message #" << message_count
                      << " (" << received_data.size() << " bytes)" << std::endl;
        }
        // If no data received within timeout, the loop continues

        // Send some data back to robot
        std::string ack_message = "ACK from receiver";
        std::vector<uint8_t> send_data(ack_message.begin(), ack_message.end());
        if (socket->send(send_data)) {
            std::cout << "Sent acknowledgment to robot" << std::endl;
        } else {
            std::cerr << "Failed to send acknowledgment" << std::endl;
        }
    }

    std::cout << "\nShutting down..." << std::endl;
    std::cout << "Total messages received: " << message_count << std::endl;

    // Disconnect socket
    socket->disconnect();

    std::cout << "Goodbye!" << std::endl;
    return 0;
}
