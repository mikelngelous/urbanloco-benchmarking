#include "robot_communication/protocol_handler.hpp"
#include <ros/ros.h>

namespace robot_communication {

bool ProtocolHandler::serializeMessage(const google::protobuf::Message& message, std::vector<uint8_t>& output) {
    std::string serialized;
    if (!message.SerializeToString(&serialized)) {
        return false;
    }
    
    output.assign(serialized.begin(), serialized.end());
    return true;
}

bool ProtocolHandler::deserializeMessage(const std::vector<uint8_t>& input, google::protobuf::Message& message) {
    std::string serialized(input.begin(), input.end());
    return message.ParseFromString(serialized);
}

} // namespace robot_communication