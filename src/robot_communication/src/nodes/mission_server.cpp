#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include <memory>
#include <signal.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <ctime>
#include "sensor_data.pb.h"

using boost::asio::ip::tcp;
using namespace robot_communication;

class MissionServer {
public:
    MissionServer(uint16_t port) : port_(port), io_context_(), acceptor_(io_context_, tcp::endpoint(tcp::v4(), port)) {
        ROS_INFO("Mission Server starting on port %d", port);
    }
    
    void start() {
        startAccept();
        
        // Run IO context in separate thread
        io_thread_ = std::make_unique<std::thread>([this]() {
            try {
                io_context_.run();
            } catch (const std::exception& e) {
                ROS_ERROR("IO context error: %s", e.what());
            }
        });
        
        ROS_INFO("Mission Server started successfully");
        ROS_INFO("Waiting for robot connections on port %d", port_);
    }
    
    void stop() {
        ROS_INFO("Stopping Mission Server");
        
        io_context_.stop();
        
        if (io_thread_ && io_thread_->joinable()) {
            io_thread_->join();
        }
    }

private:
    uint16_t port_;
    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;
    std::unique_ptr<std::thread> io_thread_;
    std::atomic<uint64_t> total_packets_received_{0};
    std::atomic<uint64_t> total_bytes_received_{0};
    
    class ClientSession : public std::enable_shared_from_this<ClientSession> {
    public:
        ClientSession(tcp::socket socket, MissionServer* server) 
            : socket_(std::move(socket)), server_(server) {}
        
        tcp::socket& socket() { return socket_; }
        
        void start() {
            auto remote = socket_.remote_endpoint();
            ROS_INFO("New client connected: %s:%d", 
                     remote.address().to_string().c_str(), remote.port());
            
            startRead();
            
            // Send initial mission command
            sendMissionCommand(MissionCommand::START_RECORDING, "frequency=10");
        }
    
    private:
        tcp::socket socket_;
        MissionServer* server_;
        std::array<uint8_t, 1024> buffer_;
        
        // Frame parsing state
        enum ReadState { READING_LENGTH, READING_PAYLOAD };
        ReadState read_state_ = READING_LENGTH;
        uint32_t expected_payload_size_ = 0;
        std::vector<uint8_t> payload_buffer_;
        std::array<uint8_t, 4> length_buffer_;
        
        void startRead() {
            if (read_state_ == READING_LENGTH) {
                readLength();
            } else {
                readPayload();
            }
        }
        
        void readLength() {
            auto self = shared_from_this();
            boost::asio::async_read(
                socket_,
                boost::asio::buffer(length_buffer_),
                [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                    if (!ec) {
                        // Parse length from network byte order
                        uint32_t network_size;
                        std::memcpy(&network_size, length_buffer_.data(), 4);
                        expected_payload_size_ = ntohl(network_size);
                        
                        // Validate payload size (max 10MB)
                        if (expected_payload_size_ > 0 && expected_payload_size_ < 10*1024*1024) {
                            payload_buffer_.resize(expected_payload_size_);
                            read_state_ = READING_PAYLOAD;
                            readPayload();
                        } else {
                            ROS_ERROR("Invalid payload size: %u bytes", expected_payload_size_);
                            read_state_ = READING_LENGTH;
                            readLength();
                        }
                    } else {
                        ROS_INFO("Client disconnected during length read: %s", ec.message().c_str());
                    }
                });
        }
        
        void readPayload() {
            auto self = shared_from_this();
            boost::asio::async_read(
                socket_,
                boost::asio::buffer(payload_buffer_),
                [this, self](boost::system::error_code ec, std::size_t length) {
                    if (!ec) {
                        handleCompleteMessage(length);
                        read_state_ = READING_LENGTH;
                        readLength();  // Start reading next message
                    } else {
                        ROS_INFO("Client disconnected during payload read: %s", ec.message().c_str());
                    }
                });
        }
        
        void handleCompleteMessage(std::size_t length) {
            server_->total_bytes_received_ += length;
            server_->total_packets_received_++;
            
            // Print received data info
            ROS_INFO_THROTTLE(5.0, "Received data: %zu bytes (Total: %lu packets, %lu bytes)",
                             length,
                             server_->total_packets_received_.load(),
                             server_->total_bytes_received_.load());
            
            // Parse and display the protobuf SensorDataPacket
            try {
                SensorDataPacket packet;
                if (packet.ParseFromArray(payload_buffer_.data(), static_cast<int>(length))) {
                        // Get current timestamp for server-side logging
                        auto now = std::chrono::system_clock::now();
                        auto time_val = std::chrono::system_clock::to_time_t(now);
                        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()) % 1000;
                        
                        struct tm* tm_info = std::localtime(&time_val);
                        char timestamp_str[100];
                        strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%d %H:%M:%S", tm_info);
                        
                        std::cout << "\n=== [" << timestamp_str << "." 
                                  << std::setfill('0') << std::setw(3) << ms.count() 
                                  << "] SENSOR DATA PACKET ===" << std::endl;
                        
                        std::cout << "Packet Info: seq=" << packet.sequence_id() 
                                  << ", frame=" << packet.frame_id()
                                  << ", size=" << length << " bytes" << std::endl;
                        
                        // Display packet timestamp (from robot)
                        if (packet.has_timestamp()) {
                            auto pkt_ts = packet.timestamp();
                            auto pkt_time_val = static_cast<std::time_t>(pkt_ts.seconds());
                            struct tm* pkt_tm = std::localtime(&pkt_time_val);
                            char pkt_timestamp_str[100];
                            strftime(pkt_timestamp_str, sizeof(pkt_timestamp_str), "%H:%M:%S", pkt_tm);
                            std::cout << "Robot Time: " << pkt_timestamp_str 
                                      << "." << std::setfill('0') << std::setw(3) 
                                      << (pkt_ts.nanos() / 1000000) << std::endl;
                        }
                        
                        // Display sensor data summary
                        std::vector<std::string> sensors;
                        if (packet.has_lidar_data()) {
                            auto& lidar = packet.lidar_data();
                            sensors.push_back("LiDAR(" + std::to_string(lidar.point_data().size()) + "B)");
                        }
                        if (packet.has_imu_data()) {
                            auto& imu = packet.imu_data();
                            sensors.push_back("IMU(acc:" + 
                                std::to_string(imu.linear_acceleration().x()) + "," +
                                std::to_string(imu.linear_acceleration().y()) + "," +
                                std::to_string(imu.linear_acceleration().z()) + ")");
                        }
                        if (packet.has_gnss_data()) {
                            auto& gnss = packet.gnss_data();
                            sensors.push_back("GNSS(lat:" + std::to_string(gnss.latitude()) + 
                                ", lon:" + std::to_string(gnss.longitude()) + 
                                ", alt:" + std::to_string(gnss.altitude()) + ")");
                        }
                        if (packet.camera_data_size() > 0) {
                            sensors.push_back("Camera(" + std::to_string(packet.camera_data_size()) + 
                                " imgs, " + std::to_string(packet.camera_data(0).data().size()) + "B each)");
                        }
                        
                        if (!sensors.empty()) {
                            std::cout << "Sensors: ";
                            for (size_t i = 0; i < sensors.size(); ++i) {
                                if (i > 0) std::cout << " | ";
                                std::cout << sensors[i];
                            }
                            std::cout << std::endl;
                        }
                        
                        std::cout << "===============================================\n" << std::endl;
                        std::cout.flush();
                        
                    } else {
                        // Get current timestamp for error logging
                        auto now = std::chrono::system_clock::now();
                        auto time_val = std::chrono::system_clock::to_time_t(now);
                        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()) % 1000;
                        
                        struct tm* tm_info = std::localtime(&time_val);
                        char timestamp_str[100];
                        strftime(timestamp_str, sizeof(timestamp_str), "%Y-%m-%d %H:%M:%S", tm_info);
                        
                        std::cout << "ERROR [" << timestamp_str << "." 
                                  << std::setfill('0') << std::setw(3) << ms.count() 
                                  << "] Failed to parse protobuf packet (" << length 
                                  << " bytes)" << std::endl;
                        std::cout.flush();
                    }
            } catch (const std::exception& e) {
                std::cout << "ERROR Exception parsing sensor data: " << e.what() << std::endl;
                std::cout.flush();
            }
        }
        
        void sendMissionCommand(MissionCommand::CommandType type, const std::string& param = "") {
            MissionCommand cmd;
            cmd.set_command_id(server_->total_packets_received_.load() + 1);
            cmd.set_type(type);
            
            auto timestamp = cmd.mutable_timestamp();
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
            auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
            
            timestamp->set_seconds(seconds.count());
            timestamp->set_nanos(static_cast<int32_t>(nanos.count()));
            
            if (!param.empty()) {
                (*cmd.mutable_parameters())["param"] = param;
            }
            
            std::string serialized;
            if (cmd.SerializeToString(&serialized)) {
                auto self = shared_from_this();
                boost::asio::async_write(
                    socket_,
                    boost::asio::buffer(serialized),
                    [this, self](boost::system::error_code ec, std::size_t /*length*/) {
                        if (ec) {
                            ROS_ERROR("Failed to send mission command: %s", ec.message().c_str());
                        }
                    });
                
                ROS_INFO("Sent mission command: type=%d", static_cast<int>(type));
            }
        }
    };
    
    void startAccept() {
        auto new_session = std::make_shared<ClientSession>(
            tcp::socket(io_context_), this);
        
        acceptor_.async_accept(
            new_session->socket(),
            [this, new_session](boost::system::error_code ec) {
                if (!ec) {
                    new_session->start();
                } else {
                    ROS_ERROR("Accept error: %s", ec.message().c_str());
                }
                
                startAccept(); // Continue accepting new connections
            });
    }
};

std::unique_ptr<MissionServer> g_server;

void signalHandler(int sig) {
    ROS_INFO("Received signal %d, shutting down server...", sig);
    if (g_server) {
        g_server->stop();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_server");
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    ros::NodeHandle nh("~");
    int port;
    nh.param("port", port, 8080);
    
    try {
        g_server = std::make_unique<MissionServer>(static_cast<uint16_t>(port));
        g_server->start();
        
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Server exception: %s", e.what());
        return 1;
    }
    
    if (g_server) {
        g_server->stop();
    }
    
    return 0;
}