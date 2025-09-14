#include <gtest/gtest.h>
#include "robot_communication/communication_manager.hpp"
#include "robot_communication/data_transfer_queue.hpp"
#include "sensor_data.pb.h"
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

namespace robot_communication {
namespace test {

class CommunicationManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.server_host = "127.0.0.1";
        config_.server_port = 8080;
        config_.connection_timeout_ms = 1000;
        config_.retry_interval_ms = 500;
        config_.max_retries = 3;
        config_.send_timeout_ms = 1000;
    }

    void TearDown() override {
        if (comm_manager_) {
            comm_manager_->stop();
        }
    }

    CommunicationManager::Config config_;
    std::unique_ptr<CommunicationManager> comm_manager_;
};

// Mock TCP Server for testing
class MockTCPServer {
private:
    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::thread server_thread_;
    bool running_;

public:
    MockTCPServer(int port) :
        acceptor_(io_context_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)),
        running_(false) {}

    ~MockTCPServer() {
        stop();
    }

    void start() {
        running_ = true;
        server_thread_ = std::thread([this]() {
            while (running_) {
                try {
                    boost::asio::ip::tcp::socket socket(io_context_);
                    acceptor_.accept(socket);

                    // Echo server - read and echo back
                    while (socket.is_open() && running_) {
                        std::vector<uint8_t> buffer(1024);
                        boost::system::error_code error;

                        size_t bytes_read = socket.read_some(boost::asio::buffer(buffer), error);
                        if (error || bytes_read == 0) break;

                        boost::asio::write(socket, boost::asio::buffer(buffer, bytes_read));
                    }
                } catch (const std::exception&) {
                    // Server closed or connection error
                    break;
                }
            }
        });
    }

    void stop() {
        running_ = false;
        acceptor_.close();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }
};

TEST_F(CommunicationManagerTest, ConstructorInitialization) {
    // Test that the constructor initializes correctly
    EXPECT_NO_THROW({
        comm_manager_ = std::make_unique<CommunicationManager>(config_);
    });
    EXPECT_NE(comm_manager_, nullptr);
}

TEST_F(CommunicationManagerTest, StartWithoutServer) {
    // Test that the manager fails gracefully when no server is available
    comm_manager_ = std::make_unique<CommunicationManager>(config_);

    bool connection_status = false;
    auto status_callback = [&connection_status](bool connected) {
        connection_status = connected;
    };

    auto command_callback = [](const std::string&) {};

    // Start should return true (async connection), but connection will fail
    EXPECT_TRUE(comm_manager_->start(status_callback, command_callback));

    // Wait a bit for connection attempt
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Connection should be false since no server is running
    EXPECT_FALSE(connection_status);
}

TEST_F(CommunicationManagerTest, ConnectionEstablishment) {
    // Start mock server
    MockTCPServer server(8080);
    server.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Let server start

    comm_manager_ = std::make_unique<CommunicationManager>(config_);

    bool connection_established = false;
    auto status_callback = [&connection_established](bool connected) {
        connection_established = connected;
    };

    auto command_callback = [](const std::string&) {};

    EXPECT_TRUE(comm_manager_->start(status_callback, command_callback));

    // Wait for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_TRUE(connection_established);

    server.stop();
}

TEST_F(CommunicationManagerTest, MessageSending) {
    MockTCPServer server(8080);
    server.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    comm_manager_ = std::make_unique<CommunicationManager>(config_);

    bool connected = false;
    auto status_callback = [&connected](bool connection_status) {
        connected = connection_status;
    };

    auto command_callback = [](const std::string&) {};

    EXPECT_TRUE(comm_manager_->start(status_callback, command_callback));

    // Wait for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_TRUE(connected);

    // Create test message using protobuf
    robot_communication::SensorDataPacket test_message;
    test_message.set_sequence_id(1);
    test_message.set_frame_id("test_frame");

    // Serialize to vector for transmission
    std::vector<uint8_t> serialized_data(test_message.ByteSizeLong());
    test_message.SerializeToArray(serialized_data.data(), serialized_data.size());

    // Send message
    bool send_result = comm_manager_->sendSensorData(test_message);
    EXPECT_TRUE(send_result);

    server.stop();
}

TEST_F(CommunicationManagerTest, StopFunctionality) {
    comm_manager_ = std::make_unique<CommunicationManager>(config_);

    auto status_callback = [](bool) {};
    auto command_callback = [](const std::string&) {};

    EXPECT_TRUE(comm_manager_->start(status_callback, command_callback));

    // Stop should not throw
    EXPECT_NO_THROW(comm_manager_->stop());

    // Multiple stops should be safe
    EXPECT_NO_THROW(comm_manager_->stop());
}

TEST_F(CommunicationManagerTest, InvalidConfiguration) {
    // Test with invalid configuration
    CommunicationManager::Config invalid_config;
    invalid_config.server_host = "";  // Empty host
    invalid_config.server_port = 0;   // Invalid port

    EXPECT_NO_THROW({
        comm_manager_ = std::make_unique<CommunicationManager>(invalid_config);
    });

    auto status_callback = [](bool) {};
    auto command_callback = [](const std::string&) {};

    // Start should handle invalid config gracefully
    EXPECT_TRUE(comm_manager_->start(status_callback, command_callback));
}

} // namespace test
} // namespace robot_communication