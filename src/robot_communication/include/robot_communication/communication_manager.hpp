#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include <functional>
#include <yaml-cpp/yaml.h>

#include "robot_communication/protocol_handler.hpp"
#include "sensor_data.pb.h"

namespace robot_communication {

/**
 * @brief High-performance TCP communication manager for robot-server communication
 * 
 * Implements custom binary protocol with protobuf serialization, LZ4 compression,
 * and adaptive QoS for real-time robot data transmission.
 */
class CommunicationManager {
public:
    using ConnectionStatusCallback = std::function<void(bool connected)>;
    using CommandCallback = std::function<void(const MissionCommand&)>;
    
    struct Config {
        std::string server_host;
        uint16_t server_port;
        size_t send_buffer_size;
        size_t receive_buffer_size;
        uint32_t heartbeat_interval_ms;
        uint32_t reconnect_interval_ms;
        uint32_t max_reconnect_attempts;
        double compression_threshold;
        bool enable_adaptive_qos;
        
        // Default constructor with safe defaults
        Config() : 
            server_host("localhost"),
            server_port(8080),
            send_buffer_size(1024 * 1024),
            receive_buffer_size(64 * 1024),
            heartbeat_interval_ms(1000),
            reconnect_interval_ms(5000),
            max_reconnect_attempts(10),
            compression_threshold(0.8),
            enable_adaptive_qos(true) {}
        
        // Load configuration from YAML file
        static Config fromYAMLFile(const std::string& file_path) {
            YAML::Node config = YAML::LoadFile(file_path);
            return fromYAML(config["communication"]);
        }
        
        // Load configuration from YAML node
        static Config fromYAML(const YAML::Node& node) {
            Config config;
            
            // Server settings
            config.server_host = node["server"]["host"].as<std::string>();
            config.server_port = node["server"]["port"].as<uint16_t>();
            
            // Performance settings
            config.send_buffer_size = static_cast<size_t>(
                node["performance"]["send_buffer_size_mb"].as<double>() * 1024 * 1024
            );
            config.receive_buffer_size = static_cast<size_t>(
                node["performance"]["receive_buffer_size_kb"].as<double>() * 1024
            );
            config.heartbeat_interval_ms = node["performance"]["heartbeat_interval_ms"].as<uint32_t>();
            config.reconnect_interval_ms = node["performance"]["reconnect_interval_ms"].as<uint32_t>();
            config.max_reconnect_attempts = node["performance"]["max_reconnect_attempts"].as<uint32_t>();
            
            // Compression settings
            config.compression_threshold = node["compression"]["threshold"].as<double>();
            
            // Adaptive QoS settings
            config.enable_adaptive_qos = node["adaptive_qos"]["enabled"].as<bool>();
            
            return config;
        }
    };
    
    /**
     * @brief Constructor
     * @param config Communication configuration
     */
    explicit CommunicationManager(const Config& config = Config());
    
    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~CommunicationManager();
    
    /**
     * @brief Start communication manager
     * @param status_callback Called when connection status changes
     * @param command_callback Called when mission commands are received
     * @return true if successful
     */
    bool start(const ConnectionStatusCallback& status_callback,
               const CommandCallback& command_callback);
    
    /**
     * @brief Stop communication manager
     */
    void stop();
    
    /**
     * @brief Send sensor data to server
     * @param packet Sensor data packet to send
     * @return true if queued successfully
     */
    bool sendSensorData(const SensorDataPacket& packet);
    
    /**
     * @brief Send robot status to server
     * @param status Robot status information
     * @return true if queued successfully
     */
    bool sendRobotStatus(const RobotStatus& status);
    
    /**
     * @brief Check if connected to server
     */
    bool isConnected() const { return is_connected_.load(); }
    
    /**
     * @brief Get communication statistics
     */
    struct Statistics {
        uint64_t bytes_sent = 0;
        uint64_t bytes_received = 0;
        uint64_t packets_sent = 0;
        uint64_t packets_received = 0;
        uint64_t packets_dropped = 0;
        double compression_ratio = 0.0;
        double average_latency_ms = 0.0;
        uint32_t reconnection_count = 0;
    };
    
    Statistics getStatistics() const;

private:
    Config config_;
    std::unique_ptr<ProtocolHandler> protocol_handler_;
    
    // Boost.Asio components
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
    std::unique_ptr<boost::asio::steady_timer> heartbeat_timer_;
    std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;
    std::unique_ptr<std::thread> io_thread_;
    
    // Callback functions
    ConnectionStatusCallback status_callback_;
    CommandCallback command_callback_;
    
    // Send queue with priority support
    struct QueuedMessage {
        std::vector<uint8_t> data;
        uint8_t priority;  // 0 = highest, 255 = lowest
        std::chrono::steady_clock::time_point timestamp;
        
        bool operator<(const QueuedMessage& other) const {
            return priority > other.priority;  // Higher priority first
        }
    };
    
    std::priority_queue<QueuedMessage> send_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::unique_ptr<std::thread> send_thread_;
    
    // Connection state
    std::atomic<bool> is_connected_{false};
    std::atomic<bool> should_stop_{false};
    std::atomic<uint32_t> reconnect_attempts_{0};
    
    // Performance tracking and adaptive QoS
    mutable std::mutex stats_mutex_;
    Statistics statistics_;
    std::chrono::steady_clock::time_point last_stats_update_;
    
    // Adaptive QoS state
    struct QosState {
        double current_bandwidth = 0.0;
        double target_bandwidth = 100.0 * 1024 * 1024;  // 100 Mbps default
        uint32_t congestion_level = 0;
        bool compression_enabled = false;
    } qos_state_;
    
    // Core methods
    void ioWorkerThread();
    void sendWorkerThread();
    void attemptConnection();
    void handleConnect(const boost::system::error_code& error);
    void startReceive();
    void handleReceive(const boost::system::error_code& error, size_t bytes_received);
    void handleSend(const boost::system::error_code& error, size_t bytes_sent);
    
    // Protocol methods
    bool serializeAndQueue(const google::protobuf::Message& message, uint8_t priority);
    void processReceivedData(const std::vector<uint8_t>& data);
    void sendHeartbeat();
    void handleHeartbeat(const boost::system::error_code& error);
    
    // Adaptive QoS methods
    void updateQosMetrics();
    void adjustCompressionLevel();
    void dropLowPriorityMessages();
    
    // Utility methods
    void updateStatistics(size_t bytes_sent, size_t bytes_received);
    void notifyConnectionStatus(bool connected);
    uint8_t calculateMessagePriority(const google::protobuf::Message& message);
};

} // namespace robot_communication