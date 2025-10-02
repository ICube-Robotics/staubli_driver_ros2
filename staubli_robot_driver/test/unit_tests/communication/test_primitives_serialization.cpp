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

#include "staubli_robot_driver/communication/serialization.hpp"

// Using declarations instead of namespace using-directive
using staubli_robot_driver::serialize_type;
using staubli_robot_driver::deserialize_type;
using staubli_robot_driver::serialize_array;
using staubli_robot_driver::deserialize_array;

// Test fixture for serialization/deserialization tests
class SerializationTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize buffer with zeroes
    std::fill(buffer.begin(), buffer.end(), 0);
  }

  // Buffer for serialization/deserialization
  std::array<uint8_t, 32> buffer;
};

// Test uint8_t serialization and deserialization
TEST_F(SerializationTest, Uint8SerializationDeserialization) {
  const uint8_t original_value = 42;
  uint8_t deserialized_value = 0;

  // Serialize
  size_t bytes_written = serialize_type(original_value, buffer.data());
  EXPECT_EQ(bytes_written, sizeof(uint8_t));

  // Deserialize
  size_t bytes_read = deserialize_type(buffer.data(), deserialized_value);
  EXPECT_EQ(bytes_read, sizeof(uint8_t));

  // Verify value
  EXPECT_EQ(original_value, deserialized_value);
}

// Test uint16_t serialization and deserialization
TEST_F(SerializationTest, Uint16SerializationDeserialization) {
  const uint16_t original_value = 12345;
  uint16_t deserialized_value = 0;

  // Serialize
  size_t bytes_written = serialize_type(original_value, buffer.data());
  EXPECT_EQ(bytes_written, sizeof(uint16_t));

  // Deserialize
  size_t bytes_read = deserialize_type(buffer.data(), deserialized_value);
  EXPECT_EQ(bytes_read, sizeof(uint16_t));

  // Verify value
  EXPECT_EQ(original_value, deserialized_value);
}

// Test uint32_t serialization and deserialization
TEST_F(SerializationTest, Uint32SerializationDeserialization) {
  const uint32_t original_value = 1234567890;
  uint32_t deserialized_value = 0;

  // Serialize
  size_t bytes_written = serialize_type(original_value, buffer.data());
  EXPECT_EQ(bytes_written, sizeof(uint32_t));

  // Deserialize
  size_t bytes_read = deserialize_type(buffer.data(), deserialized_value);
  EXPECT_EQ(bytes_read, sizeof(uint32_t));

  // Verify value
  EXPECT_EQ(original_value, deserialized_value);
}

// Test float serialization and deserialization
TEST_F(SerializationTest, FloatSerializationDeserialization) {
  const float original_value = 3.14159f;
  float deserialized_value = 0.0f;

  // Serialize
  size_t bytes_written = serialize_type(original_value, buffer.data());
  EXPECT_EQ(bytes_written, sizeof(float));

  // Deserialize
  size_t bytes_read = deserialize_type(buffer.data(), deserialized_value);
  EXPECT_EQ(bytes_read, sizeof(float));

  // Verify value (using near for floating point comparison)
  EXPECT_NEAR(original_value, deserialized_value, 1e-5);
}

// Test double serialization and deserialization (note: converts to float)
TEST_F(SerializationTest, DoubleSerializationDeserialization) {
  const double original_value = 2.71828182846;
  double deserialized_value = 0.0;

  // Serialize
  size_t bytes_written = serialize_type(original_value, buffer.data());
  EXPECT_EQ(bytes_written, sizeof(float));  // Double is serialized as float

  // Deserialize
  size_t bytes_read = deserialize_type(buffer.data(), deserialized_value);
  EXPECT_EQ(bytes_read, sizeof(float));  // Float size since double is deserialized from float

  // Verify value (with larger epsilon due to float/double conversion)
  EXPECT_NEAR(original_value, deserialized_value, 1e-5);

  // Additional test to verify precision loss
  float expected_float_value = static_cast<float>(original_value);
  EXPECT_NEAR(expected_float_value, deserialized_value, 1e-5);
}

// Test array serialization and deserialization
TEST_F(SerializationTest, ArraySerializationDeserialization) {
  const std::array<uint32_t, 3> original_array = {123, 456, 789};
  std::array<uint32_t, 3> deserialized_array = {0, 0, 0};

  // Serialize
  size_t bytes_written = serialize_array(original_array, buffer.data());
  EXPECT_EQ(bytes_written, original_array.size() * sizeof(uint32_t));

  // Deserialize
  size_t bytes_read = deserialize_array(buffer.data(), deserialized_array);
  EXPECT_EQ(bytes_read, deserialized_array.size() * sizeof(uint32_t));

  // Verify values
  for (size_t i = 0; i < original_array.size(); ++i) {
    EXPECT_EQ(original_array[i], deserialized_array[i]);
  }
}

// Test float array serialization and deserialization
TEST_F(SerializationTest, FloatArraySerializationDeserialization) {
  const std::array<float, 4> original_array = {1.1f, 2.2f, 3.3f, 4.4f};
  std::array<float, 4> deserialized_array = {0.0f, 0.0f, 0.0f, 0.0f};

  // Serialize
  size_t bytes_written = serialize_array(original_array, buffer.data());
  EXPECT_EQ(bytes_written, original_array.size() * sizeof(float));

  // Deserialize
  size_t bytes_read = deserialize_array(buffer.data(), deserialized_array);
  EXPECT_EQ(bytes_read, deserialized_array.size() * sizeof(float));

  // Verify values
  for (size_t i = 0; i < original_array.size(); ++i) {
    EXPECT_NEAR(original_array[i], deserialized_array[i], 1e-5);
  }
}

// Main function to run the tests
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
