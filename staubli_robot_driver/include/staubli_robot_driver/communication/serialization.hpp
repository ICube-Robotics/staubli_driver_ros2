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

#ifndef STAUBLI_ROBOT_DRIVER__COMMUNICATION__SERIALIZATION_HPP_
#define STAUBLI_ROBOT_DRIVER__COMMUNICATION__SERIALIZATION_HPP_

#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <cstring>


// Ensure code is compiled on a little-endian system
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
    #if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
        #error "This code requires a little-endian system"
    #endif
#elif defined(__BYTE_ORDER) && defined(__LITTLE_ENDIAN)
    #if __BYTE_ORDER != __LITTLE_ENDIAN
        #error "This code requires a little-endian system"
    #endif
#elif defined(_BYTE_ORDER) && defined(_LITTLE_ENDIAN)
    #if _BYTE_ORDER != _LITTLE_ENDIAN
        #error "This code requires a little-endian system"
    #endif
#else
    #error "Unknown endianness, cannot determine if system is little-endian"
#endif

namespace staubli_robot_driver {

// Serialization specializations

// size_t serialize_type(const bool& value, uint8_t* buffer)
// {
//     buffer[0] = value ? 0xFF : 0x00;
//     return sizeof(uint8_t);
// }

inline size_t serialize_type(const uint8_t& value, uint8_t* buffer)
{
    buffer[0] = value;
    return sizeof(value);
}

inline size_t serialize_type(const uint16_t& value, uint8_t* buffer)
{
    memcpy(buffer, &value, sizeof(value));
    return sizeof(value);
}

inline size_t serialize_type(const uint32_t& value, uint8_t* buffer)
{
    memcpy(buffer, &value, sizeof(value));
    return sizeof(value);
}

inline size_t serialize_type(const float& value, uint8_t* buffer)
{
    memcpy(buffer, &value, sizeof(value));
    return sizeof(value);
}

/// Specialization for double: serialize_type as float to save space
inline size_t serialize_type(const double& value, uint8_t* buffer)
{
    float value_float = static_cast<float>(value);
    return serialize_type(value_float, buffer);
}

// Deserialization specializations

// size_t deserialize_type(const uint8_t* buffer, bool& value)
// {
//     value = buffer[0] != 0;
//     return sizeof(uint8_t);
// }

inline size_t deserialize_type(const uint8_t* buffer, uint8_t& value)
{
    value = buffer[0];
    return sizeof(value);
}

inline size_t deserialize_type(const uint8_t* buffer, uint16_t& value)
{
    memcpy(&value, buffer, sizeof(value));
    return sizeof(value);
}

inline size_t deserialize_type(const uint8_t* buffer, uint32_t& value)
{
    memcpy(&value, buffer, sizeof(value));
    return sizeof(value);
}

inline size_t deserialize_type(const uint8_t* buffer, float& value)
{
    memcpy(&value, buffer, sizeof(value));
    return sizeof(value);
}

/// Specialization for double: deserialize_type from float
inline size_t deserialize_type(const uint8_t* buffer, double& value)
{
    float value_float;
    size_t bytes_read = deserialize_type(buffer, value_float);
    value = static_cast<double>(value_float);
    return bytes_read;
}

// Array serialization/deserialization

/**
 * @brief Serialize an array of type T into a byte buffer
 * @warning The size of the buffer is not checked...
 * @tparam T Underlying type of the array elements
 * @tparam N Number of elements in the array
 * @param arr Array to serialize
 * @param buffer Buffer to write to
 * @return size_t Number of bytes written to the buffer
 */
template<typename T, std::size_t N>
size_t serialize_array(const std::array<T, N>& arr, uint8_t* buffer)
{
    size_t offset = 0;
    for (const auto& item : arr) {
        offset += serialize_type(item, buffer + offset);
    }
    return offset;
}

/**
 * @brief Deserialize an array of type T from a byte buffer
 * @warning The size of the buffer is not checked...
 * @tparam T Underlying type of the array elements
 * @tparam N Number of elements in the array
 * @param buffer Buffer to read from
 * @param arr Array to deserialize
 * @return size_t Number of bytes read from the buffer
 */
template<typename T, std::size_t N>
size_t deserialize_array(const uint8_t* buffer, std::array<T, N>& arr)
{
    size_t offset = 0;
    for (auto& item : arr) {
        offset += deserialize_type(buffer + offset, item);
    }
    return offset;
}

}  // namespace staubli_robot_driver

#endif  // STAUBLI_ROBOT_DRIVER__COMMUNICATION__SERIALIZATION_HPP_
