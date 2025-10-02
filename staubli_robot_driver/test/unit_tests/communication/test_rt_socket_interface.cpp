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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

#include "staubli_robot_driver/communication/socket.hpp"
#include "staubli_robot_driver/communication/udp_socket.hpp"
#include "staubli_robot_driver/real_time_socket_subscriber.hpp"
#include "staubli_robot_driver/real_time_socket_interface.hpp"
#include "staubli_robot_driver/communication/messages.hpp"
#include "staubli_robot_driver/communication/protocol.hpp"

using staubli_robot_driver::Socket;
using staubli_robot_driver::SocketFactory;
using staubli_robot_driver::RealTimeSocketSubscriber;
using staubli_robot_driver::RealTimeSocketInterface;
using staubli_robot_driver::MessageStatus;
using staubli_robot_driver::RobotStateMessage;
using staubli_robot_driver::RobotCommandMessage;

// Test fixture for RealTimeSocketSubscriber and RealTimeSocketInterface
class RTSocketInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a socket pair for testing
    socket1_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);
    socket2_ = SocketFactory::create(SocketFactory::ProtocolType::UDP);

    // Connect sockets
    const std::string loopback_address = "127.0.0.1";
    const uint16_t port1 = 12345;
    const uint16_t port2 = 12346;

    socket1_->connect(loopback_address, port2, port1);
    socket2_->connect(loopback_address, port1, port2);
  }

  void TearDown() override {
    socket1_->stop_receive_thread();
    socket2_->stop_receive_thread();
    // Stop receive thread and disconnect sockets
    socket1_->disconnect();
    socket2_->disconnect();
  }

  std::shared_ptr<Socket> socket1_;  // Sending socket
  std::shared_ptr<Socket> socket2_;  // Receiving socket
};

class RTSocketSubscriberTest : public RTSocketInterfaceTest {
public:
  void SetUp() override {
    RTSocketInterfaceTest::SetUp();
  }

    void TearDown() override {
        subscriber_.reset();
        RTSocketInterfaceTest::TearDown();
    }

  std::shared_ptr<RealTimeSocketSubscriber<RobotStateMessage>> subscriber_;
};

/* ------------------------------------------
 *      Test RealTimeSocketSubscriber
   ------------------------------------------ */

TEST_F(RTSocketSubscriberTest, Initialization) {
  // Create a subscriber
  std::cout << "Creating subscriber..." << std::endl;
  subscriber_ = \
    std::make_shared<RealTimeSocketSubscriber<RobotStateMessage>>(socket1_);

  // Check if initialization was successful
  ASSERT_NE(subscriber_, nullptr);
}

TEST_F(RTSocketSubscriberTest, ReceiveMessage) {
  // Create a subscriber
  subscriber_ = \
    std::make_shared<RealTimeSocketSubscriber<RobotStateMessage>>(socket1_);

  // Create a test message
  RobotStateMessage send_msg;
  send_msg.header.sequence_number = 1;
  // Fill message with test data
  send_msg.joint_positions = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

  // Serialize message to a buffer
  std::vector<uint8_t> buffer;
  buffer.resize(send_msg.get_serialized_size());

  // Serialize using the protocol
  bool result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);

  // Send the serialized message
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  // Add a small delay to ensure the message is sent before reading
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Read the message
  RobotStateMessage recv_msg;
  MessageStatus status;
  result = subscriber_->read_message(recv_msg, status);

  // Check if message was received correctly
  ASSERT_TRUE(result);
  EXPECT_EQ(recv_msg.header.sequence_number, send_msg.header.sequence_number);
  EXPECT_EQ(recv_msg.joint_positions, send_msg.joint_positions);
  EXPECT_EQ(status.lost_packages, 0);
  EXPECT_EQ(status.new_message, true);
  // Time since received should be very small (<0.5 second)
  EXPECT_TRUE(status.time_since_received.seconds() < 0.5);

  // Read again, no new message, but not yet stale
  result = subscriber_->read_message(recv_msg, status);
  ASSERT_TRUE(result);
  EXPECT_EQ(status.lost_packages, 0);
  EXPECT_EQ(status.new_message, false);
  EXPECT_TRUE(status.time_since_received.seconds() < 1.0);

  // Wait for staleness (assuming 1 second for test)
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Now the message should be stale
  result = subscriber_->read_message(recv_msg, status);
  ASSERT_TRUE(result);
  EXPECT_EQ(status.lost_packages, 0);
  EXPECT_EQ(status.new_message, false);
  EXPECT_TRUE(status.time_since_received.seconds() > 1.0);
}

// Test lost package detection
TEST_F(RTSocketSubscriberTest, LostPackageDetection) {
  // Create a subscriber
  subscriber_ = \
    std::make_shared<RealTimeSocketSubscriber<RobotStateMessage>>(socket1_);

  // Create a test message
  RobotStateMessage send_msg;
  send_msg.header.sequence_number = 7;
  // Fill message with test data
  send_msg.joint_positions = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

  // Serialize message to a buffer
  std::vector<uint8_t> buffer;
  buffer.resize(send_msg.get_serialized_size());

  // Send first serialized message
  bool result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  // Add a small delay to ensure the message is sent before reading
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Read first message
  RobotStateMessage recv_msg;
  MessageStatus status;
  result = subscriber_->read_message(recv_msg, status);
  ASSERT_TRUE(result);
  ASSERT_TRUE(status.new_message);

  // Check if message was received correctly
  EXPECT_EQ(recv_msg.header.sequence_number, send_msg.header.sequence_number);
  EXPECT_EQ(recv_msg.joint_positions, send_msg.joint_positions);
  // No lost packages on first read
  EXPECT_EQ(status.lost_packages, 0);

  // Send message n°9 (skipping one to simulate a lost package)
  send_msg.header.sequence_number += 2;
  result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  ASSERT_TRUE(socket1_->is_connected());
  ASSERT_TRUE(socket1_->is_receiving());

  // Add a small delay to ensure the message is sent before reading
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  result = subscriber_->read_message(recv_msg, status);
  ASSERT_TRUE(result);
  EXPECT_EQ(status.new_message, true);
  EXPECT_EQ(recv_msg.header.sequence_number, send_msg.header.sequence_number);
  EXPECT_EQ(status.lost_packages, 1);

  // Send message three without reading to simulate multiple lost packages
  send_msg.header.sequence_number += 1;
  result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  send_msg.header.sequence_number += 1;
  result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  send_msg.header.sequence_number += 1;
  result = send_msg.serialize(buffer);
  ASSERT_TRUE(result);
  result = socket2_->send(buffer);
  ASSERT_TRUE(result);

  // Add a small delay to ensure the message is sent before reading
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  result = subscriber_->read_message(recv_msg, status);
  ASSERT_TRUE(result);
  EXPECT_EQ(status.lost_packages, 2);
}


/* ------------------------------------------
 *      Test RealTimeSocketInterface
   ------------------------------------------ */

TEST_F(RTSocketInterfaceTest, Initialization) {
    auto interface1 = std::make_shared<
        RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>>(socket1_);

    ASSERT_NE(interface1, nullptr);
}

// Test bidirectional communication
TEST_F(RTSocketInterfaceTest, BidirectionalCommunication) {
    // Create test messages
    RobotStateMessage state_msg;
    state_msg.header.sequence_number = 10;
    state_msg.joint_positions = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f};

    RobotCommandMessage cmd_msg;
    cmd_msg.header.sequence_number = 20;
    cmd_msg.command_type = staubli_robot_driver::CommandType::JOINT_POSITION;
    cmd_msg.command_reference = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};

    // Create interfaces
    auto interface1 = std::make_shared<
        RealTimeSocketInterface<RobotStateMessage, RobotCommandMessage>>(socket1_);

    auto interface2 = std::make_shared<
        RealTimeSocketInterface<RobotCommandMessage, RobotStateMessage>>(socket2_);

    // Send state message from interface2 to interface1
    bool result = interface2->send_message(state_msg);
    ASSERT_TRUE(result);

    // Send command message from interface1 to interface2
    result = interface1->send_message(cmd_msg);
    ASSERT_TRUE(result);

    // Add a small delay to ensure messages are sent before reading
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Receive state message on interface1
    RobotStateMessage recv_state_msg;
    MessageStatus status;
    result = interface1->read_message(recv_state_msg, status);
    ASSERT_TRUE(result);
    EXPECT_EQ(recv_state_msg.header.sequence_number, state_msg.header.sequence_number);
    EXPECT_EQ(recv_state_msg.joint_positions, state_msg.joint_positions);

    // Receive command message on interface2
    RobotCommandMessage recv_cmd_msg;
    result = interface2->read_message(recv_cmd_msg, status);
    ASSERT_TRUE(result);
    EXPECT_EQ(recv_cmd_msg.header.sequence_number, cmd_msg.header.sequence_number);
    EXPECT_EQ(recv_cmd_msg.command_reference, cmd_msg.command_reference);
    EXPECT_EQ(recv_cmd_msg.command_type, cmd_msg.command_type);
}

// Main function
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
