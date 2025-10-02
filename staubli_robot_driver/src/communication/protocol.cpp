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


#include <arpa/inet.h>  // For htonl, htons, etc.

#include <cstring>
#include <iostream>
#include <stdexcept>

#include "staubli_robot_driver/communication/protocol.hpp"
#include "staubli_robot_driver/communication/serialization.hpp"

namespace staubli_robot_driver {

// FrameHeader implementation

bool FrameHeader::serialize(uint8_t* buffer, size_t buffer_size) const
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "FrameHeader::serialize: "
            << "Buffer size is too small for FrameHeader" << std::endl;
        return false;
    }

    size_t offset = 0;

    offset += serialize_type(magic_number, buffer + offset);
    offset += serialize_type(protocol_version, buffer + offset);
    offset += serialize_type(message_type, buffer + offset);
    offset += serialize_type(sequence_number, buffer + offset);
    offset += serialize_type(payload_size, buffer + offset);

    return true;
}

bool FrameHeader::deserialize(uint8_t* buffer, size_t buffer_size)
{
    if (buffer_size < get_serialized_size()) {
        std::cerr << "FrameHeader::deserialize: "
            << "Buffer size is too small for FrameHeader" << std::endl;
        return false;
    }

    size_t offset = 0;

    offset += deserialize_type(buffer + offset, magic_number);
    offset += deserialize_type(buffer + offset, protocol_version);
    offset += deserialize_type(buffer + offset, message_type);
    offset += deserialize_type(buffer + offset, sequence_number);
    offset += deserialize_type(buffer + offset, payload_size);

    if (magic_number != MAGIC_NUMBER) {
        std::cerr << "FrameHeader::deserialize: "
            << "Invalid magic number '" << magic_number
            << "' in FrameHeader" << std::endl;
        return false;
    }

    return true;
}

// Message implementation

bool Message::serialize(std::vector<uint8_t>& data) const
{
    if (data.capacity() < get_serialized_size()) {
        return false;
    }
    data.resize(get_serialized_size());
    return serialize(data.data(), data.size());
}

bool Message::deserialize(std::vector<uint8_t>& data)
{
    if (data.size() < get_serialized_size()) {
        return false;
    }
    return deserialize(data.data(), data.size());
}

}  // namespace staubli_robot_driver
