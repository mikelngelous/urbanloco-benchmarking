#include <gtest/gtest.h>
#include "robot_communication/protocol_handler.hpp"
#include <vector>
#include <cstdint>

namespace robot_communication {
namespace test {

class ProtocolHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        protocol_handler_ = std::make_unique<ProtocolHandler>();
    }

    void TearDown() override {
        protocol_handler_.reset();
    }

    std::unique_ptr<ProtocolHandler> protocol_handler_;
};

TEST_F(ProtocolHandlerTest, InitializationTest) {
    // Test that the protocol handler initializes correctly
    EXPECT_NE(protocol_handler_, nullptr);
}

TEST_F(ProtocolHandlerTest, BasicSerializationProtocol) {
    // Test basic protobuf message serialization
    robot_communication::SensorDataPacket test_msg;
    test_msg.set_sequence_id(123);
    test_msg.set_frame_id("test_frame");

    std::vector<uint8_t> serialized_data;
    EXPECT_TRUE(protocol_handler_->serializeMessage(test_msg, serialized_data));

    // Verify that there is serialized data
    EXPECT_GT(serialized_data.size(), 0);

    // Test deserialization
    robot_communication::SensorDataPacket deserialized_msg;
    EXPECT_TRUE(protocol_handler_->deserializeMessage(serialized_data, deserialized_msg));

    // Verify that data was preserved
    EXPECT_EQ(deserialized_msg.sequence_id(), 123);
    EXPECT_EQ(deserialized_msg.frame_id(), "test_frame");
}

TEST_F(ProtocolHandlerTest, EmptyMessageHandling) {
    // Test empty message handling
    robot_communication::SensorDataPacket empty_msg;

    std::vector<uint8_t> serialized_empty;
    EXPECT_TRUE(protocol_handler_->serializeMessage(empty_msg, serialized_empty));

    // Should have some serialized data (protobuf metadata)
    EXPECT_GT(serialized_empty.size(), 0);

    // Test deserialization
    robot_communication::SensorDataPacket deserialized_empty;
    EXPECT_TRUE(protocol_handler_->deserializeMessage(serialized_empty, deserialized_empty));
}

TEST_F(ProtocolHandlerTest, LargeMessageHandling) {
    // Test with large message with simulated LiDAR data
    robot_communication::SensorDataPacket large_msg;
    large_msg.set_sequence_id(999);
    large_msg.set_frame_id("large_test");

    // Simulate large LiDAR data
    auto* lidar_data = large_msg.mutable_lidar_data();
    std::string large_payload(10000, 'A'); // 10KB of data
    lidar_data->set_point_cloud_data(large_payload);

    std::vector<uint8_t> serialized_large;
    EXPECT_TRUE(protocol_handler_->serializeMessage(large_msg, serialized_large));

    // Should be able to serialize large message
    EXPECT_GT(serialized_large.size(), 10000);

    // Parse back
    robot_communication::SensorDataPacket parsed_large;
    EXPECT_TRUE(protocol_handler_->deserializeMessage(serialized_large, parsed_large));
    EXPECT_EQ(parsed_large.sequence_id(), 999);
}

TEST_F(ProtocolHandlerTest, InvalidDataHandling) {
    // Test deserialization of invalid data
    std::vector<uint8_t> invalid_data = {0x01, 0x02, 0xFF, 0xAA}; // Random bytes

    robot_communication::SensorDataPacket invalid_msg;
    EXPECT_FALSE(protocol_handler_->deserializeMessage(invalid_data, invalid_msg));
}

} // namespace test
} // namespace robot_communication