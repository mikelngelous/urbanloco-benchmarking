#include <ros/ros.h>
#include <boost/asio.hpp>
#include <thread>
#include <memory>
#include <signal.h>
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
        
        void startRead() {
            auto self = shared_from_this();
            socket_.async_read_some(
                boost::asio::buffer(buffer_),
                [this, self](boost::system::error_code ec, std::size_t length) {
                    if (!ec) {
                        handleRead(length);
                        startRead();
                    } else {
                        ROS_INFO("Client disconnected: %s", ec.message().c_str());
                    }
                });
        }
        
        void handleRead(std::size_t length) {
            server_->total_bytes_received_ += length;
            server_->total_packets_received_++;
            
            // Print received data info
            ROS_INFO_THROTTLE(1.0, "Server received: %zu bytes (Total: %lu packets, %lu bytes)",
                             length, 
                             server_->total_packets_received_.load(),
                             server_->total_bytes_received_.load());
            
            // Here we would parse the protobuf SensorDataPacket
            // For demo, just acknowledge receipt
            if (length > 4) { // Basic validity check
                std::cout << "Received sensor data packet of " << length << " bytes" << std::endl;
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