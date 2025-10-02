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

#include <gtest/gtest.h>
#include <future>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <utility>  // For std::pair

#include "staubli_robot_driver/communication/socket.hpp"
#include "staubli_robot_driver/communication/udp_socket.hpp"

using staubli_robot_driver::Socket;
using staubli_robot_driver::SocketFactory;
using staubli_robot_driver::UDPSocket;
using staubli_robot_driver::UDPSocketFactory;

// Struct to hold socket type parameters
struct SocketTypeParam {
  const char* name;
  SocketFactory::ProtocolType type;
  uint16_t default_port1;
  uint16_t default_port2;
};

// Test fixture for parameterized socket tests
class SocketTest : public ::testing::TestWithParam<SocketTypeParam> {
protected:
  void SetUp() override {
    // Get the socket type from the parameter
    socket_type_ = GetParam().type;
    port1_ = GetParam().default_port1;
    port2_ = GetParam().default_port2;

    // Create a socket pair for testing
    socket1_ = SocketFactory::create(socket_type_);
    socket2_ = SocketFactory::create(socket_type_);

    // Initialize test data
    test_data_.resize(128);
    for (size_t i = 0; i < test_data_.size(); ++i) {
      test_data_[i] = static_cast<uint8_t>(i & 0xFF);
    }
  }

  void TearDown() override {
    // Ensure sockets are disconnected
    if (socket1_ && socket1_->is_connected()) {
      socket1_->disconnect();
    }
    if (socket2_ && socket2_->is_connected()) {
      socket2_->disconnect();
    }
  }

  // Helper method to connect sockets in a loopback configuration
  bool ConnectSocketsForLoopback() {
    bool result1 = socket1_->connect(loopback_address_, port2_, port1_);
    bool result2 = socket2_->connect(loopback_address_, port1_, port2_);

    return result1 && result2;
  }

  SocketFactory::ProtocolType socket_type_;
  std::string loopback_address_ = "127.0.0.1";
  uint16_t port1_;
  uint16_t port2_;
  std::shared_ptr<Socket> socket1_;
  std::shared_ptr<Socket> socket2_;
  std::vector<uint8_t> test_data_;
};

// Test socket factory
TEST_P(SocketTest, CreateSocket) {
  auto socket = SocketFactory::create(GetParam().type);
  ASSERT_NE(socket, nullptr);
  EXPECT_FALSE(socket->is_connected());
}

// Test socket connection
TEST_P(SocketTest, ConnectDisconnect) {
  const std::string loopback_address = "127.0.0.1";
  const uint16_t local_port = port1_;
  const uint16_t remote_port = port2_;

  EXPECT_FALSE(socket1_->is_connected());

  bool result = socket1_->connect(loopback_address, remote_port, local_port);
  EXPECT_TRUE(result);
  EXPECT_TRUE(socket1_->is_connected());

  result = socket1_->disconnect();
  EXPECT_TRUE(result);
  EXPECT_FALSE(socket1_->is_connected());
}

// Define the test cases for all socket types
INSTANTIATE_TEST_CASE_P(
  SocketTypes,
  SocketTest,
  ::testing::Values(
    SocketTypeParam{"UDP", SocketFactory::ProtocolType::UDP, 12345, 12346}
    // Add TCP when implemented:
    // SocketTypeParam{"TCP", SocketFactory::ProtocolType::TCP, 8080, 8081}
  ),
  [](const ::testing::TestParamInfo<SocketTest::ParamType>& info) {
    return info.param.name;  // Use the name field for test names
  }
);

// The tests that were previously in UDPSocketTest will now use the parameterized SocketTest fixture

// Test continuous receiving with a callback
TEST_P(SocketTest, ReceiveCallback) {
  // Set up the sockets for communication
  ASSERT_TRUE(ConnectSocketsForLoopback());

  // Create a promise/future for the callback
  std::promise<bool> callback_promise;
  std::future<bool> callback_future = callback_promise.get_future();

  // Set up a receive callback
  std::vector<uint8_t> received_data;
  auto callback = [&received_data, &callback_promise]
                  (std::vector<uint8_t>& data, size_t bytes_transferred) {
    std::cout << "Data received in callback: " << bytes_transferred << " bytes" << std::endl;
    received_data.resize(bytes_transferred);
    std::copy(data.begin(), data.begin() + bytes_transferred, received_data.begin());
    callback_promise.set_value(true);
  };

  // Start receive thread
  bool result = socket2_->start_receive_thread(callback);
  EXPECT_TRUE(result);

  // Add a small delay to ensure the receive thread is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Send data
  std::cout << "Sending " << test_data_.size() << " bytes of data..." << std::endl;
  bool send_result = socket1_->send(test_data_);
  EXPECT_TRUE(send_result);

  // Wait for callback to be called
  std::cout << "Waiting for data to be received..." << std::endl;
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  EXPECT_EQ(status, std::future_status::ready) << "Callback was not called within timeout";
  std::cout << "Data received!" << std::endl;

  // Stop receive thread
  std::cout << "Stopping receive thread..." << std::endl;
  result = socket2_->stop_receive_thread();
  std::cout << "Receive thread stopped!" << std::endl;
  EXPECT_TRUE(result);

  // Check received data
  EXPECT_EQ(test_data_.size(), received_data.size());
  for (size_t i = 0; i < test_data_.size(); ++i) {
    EXPECT_EQ(test_data_[i], received_data[i]);
  }
}

// Test receive multiple messages in sequence
TEST_P(SocketTest, ReceiveMultipleMessages) {
    // Set up the sockets for communication
    ASSERT_TRUE(ConnectSocketsForLoopback());
    ASSERT_TRUE(socket1_->is_connected());
    ASSERT_TRUE(socket2_->is_connected());

    // Create atomic counter for received messages
    std::atomic<size_t> received_count(0);

    // Set up a receive callback
    auto receive_callback = [&received_count](
    std::vector<uint8_t>& /*data*/, size_t /*bytes_transferred*/)
    {
        received_count++;
    };

    // Start receive thread
    bool result = socket2_->start_receive_thread(receive_callback);
    EXPECT_TRUE(result);

    // Send multiple messages
    const size_t total_messages = 50;
    for (size_t i = 0; i < total_messages; ++i) {
        bool send_result = socket1_->send(test_data_);
        EXPECT_TRUE(send_result);
        // Small delay between sends
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Wait for all messages to be received or timeout
    auto start_time = std::chrono::steady_clock::now();
    while (received_count.load() < total_messages) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed > std::chrono::seconds(1)) {
            FAIL() << "Timeout waiting for all messages to be received";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_EQ(received_count.load(), total_messages);

    // Stop receive thread
    result = socket2_->stop_receive_thread();
    EXPECT_TRUE(result);
}

// Test Reconnect after disconnect
TEST_P(SocketTest, ReconnectAfterDisconnect) {
  // Set up the sockets for communication
  ASSERT_TRUE(ConnectSocketsForLoopback());
  EXPECT_TRUE(socket1_->is_connected());

  // Disconnect
  bool result = socket1_->disconnect();
  EXPECT_TRUE(result);
  EXPECT_FALSE(socket1_->is_connected());

  // Reconnect
  result = socket1_->connect(loopback_address_, port2_, port1_);
  EXPECT_TRUE(result);
  EXPECT_TRUE(socket1_->is_connected());

  // Receive on socket2 to ensure it's still functional
  std::vector<uint8_t> received_data;
  std::promise<bool> callback_promise;
  std::future<bool> callback_future = callback_promise.get_future();
  auto callback = [&received_data, &callback_promise]
                  (std::vector<uint8_t>& data, size_t bytes_transferred) {
    std::cout << "Data received in callback: " << bytes_transferred << " bytes" << std::endl;
    received_data.resize(bytes_transferred);
    std::copy(data.begin(), data.begin() + bytes_transferred, received_data.begin());
    callback_promise.set_value(true);
  };
  result = socket2_->start_receive_thread(callback);
  EXPECT_TRUE(result);

  // Test send after reconnect
  result = socket1_->send(test_data_);
  EXPECT_TRUE(result);

  // Wait for callback to be called
  std::cout << "Waiting for data to be received..." << std::endl;
  auto status = callback_future.wait_for(std::chrono::seconds(5));
  EXPECT_EQ(status, std::future_status::ready) << "Callback was not called within timeout";
  std::cout << "Data received!" << std::endl;

  // Check received data
  EXPECT_EQ(test_data_.size(), received_data.size());
  for (size_t i = 0; i < test_data_.size(); ++i) {
    EXPECT_EQ(test_data_[i], received_data[i]);
  }

  // Cleanup
  result = socket2_->stop_receive_thread();
  EXPECT_TRUE(result);
  result = socket1_->disconnect();
  EXPECT_TRUE(result);
  result = socket2_->disconnect();
  EXPECT_TRUE(result);
}

// Test receive_once with timeout
TEST_P(SocketTest, ReceiveOnceWithTimeout) {
  // Set up the sockets for communication
  ASSERT_TRUE(ConnectSocketsForLoopback());

  // Send data
  bool send_result = socket1_->send(test_data_);
  EXPECT_TRUE(send_result);

  // Receive data with timeout
  std::vector<uint8_t> received_data;
  bool receive_result = socket2_->receive_once(2000, received_data);  // 2 seconds timeout
  EXPECT_TRUE(receive_result);
  EXPECT_EQ(test_data_.size(), received_data.size());
  for (size_t i = 0; i < test_data_.size(); ++i) {
    EXPECT_EQ(test_data_[i], received_data[i]);
  }

  // Cleanup
  bool result = socket1_->disconnect();
  EXPECT_TRUE(result);
  result = socket2_->disconnect();
  EXPECT_TRUE(result);
}

// Test error handling - try to send when disconnected
TEST_P(SocketTest, SendWhenDisconnected) {
  // Don't connect the socket
  ASSERT_FALSE(socket1_->is_connected());

  // Try to send data
  bool result = socket1_->send(test_data_);
  EXPECT_FALSE(result);
}

// Test error handling - double connect/disconnect
TEST_P(SocketTest, DoubleConnect) {
  const std::string loopback_address = "127.0.0.1";
  const uint16_t local_port = port1_;
  const uint16_t remote_port = port2_;

  // First connect should succeed
  bool result = socket1_->connect(loopback_address, remote_port, local_port);
  EXPECT_TRUE(result);

  // Second connect should also succeed (implementation should disconnect first)
  result = socket1_->connect(loopback_address, remote_port, local_port + 1);
  EXPECT_TRUE(result);

  // Cleanup
  result = socket1_->disconnect();
  EXPECT_TRUE(result);

  // Second disconnect should succeed (idempotent)
  result = socket1_->disconnect();
  EXPECT_TRUE(result);
}

// Test error handling - double start/stop receive thread
TEST_P(SocketTest, DoubleStartReceiveThread) {
    // Set up the sockets for communication
    ASSERT_TRUE(ConnectSocketsForLoopback());
    // Create a promise/future for the callback
    std::promise<bool> callback_promise;
    std::future<bool> callback_future = callback_promise.get_future();
    auto callback = [&callback_promise](
    std::vector<uint8_t>& /*data*/, size_t /*bytes_transferred*/)
    {
        callback_promise.set_value(true);
    };
    // Start receive thread
    bool result1 = socket1_->start_receive_thread(callback);
    EXPECT_TRUE(result1);
    // Second start should fail
    bool result2 = socket1_->start_receive_thread(callback);
    EXPECT_FALSE(result2);
    // Stop receive thread
    result1 = socket1_->stop_receive_thread();
    EXPECT_TRUE(result1);
    // Second stop should succeed (idempotent)
    result2 = socket1_->stop_receive_thread();
    EXPECT_TRUE(result2);
}

// Main function to run all tests
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
