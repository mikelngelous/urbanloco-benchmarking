#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <google/protobuf/message.h>

namespace robot_communication {

/**
 * @brief Protocol handler for serializing/deserializing messages
 */
class ProtocolHandler {
public:
    ProtocolHandler() = default;
    virtual ~ProtocolHandler() = default;
    
    /**
     * @brief Serialize protobuf message to byte vector
     */
    virtual bool serializeMessage(const google::protobuf::Message& message, std::vector<uint8_t>& output);
    
    /**
     * @brief Deserialize byte vector to protobuf message
     */
    virtual bool deserializeMessage(const std::vector<uint8_t>& input, google::protobuf::Message& message);
};

} // namespace robot_communication