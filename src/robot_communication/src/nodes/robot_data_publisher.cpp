#include <ros/ros.h>
#include <signal.h>
#include "robot_communication/hybrid_synchronizer.hpp"
#include "robot_communication/communication_manager.hpp"

using namespace robot_communication;

class RobotDataPublisherNode {
public:
    RobotDataPublisherNode() : nh_("~") {
        ROS_INFO("Starting Robot Data Publisher Node");
        
        // Load parameters from YAML configuration
        loadParameters();
        
        // Initialize components
        HybridSynchronizer::Config sync_config = createSynchronizerConfig();
        synchronizer_ = std::make_unique<HybridSynchronizer>(nh_, sync_config);
        
        CommunicationManager::Config comm_config = createCommunicationConfig();
        comm_manager_ = std::make_unique<CommunicationManager>(comm_config);
    }
    
    bool start() {
        // Start communication manager
        bool comm_started = comm_manager_->start(
            [this](bool connected) { onConnectionStatus(connected); },
            [this](const MissionCommand& cmd) { onMissionCommand(cmd); }
        );
        
        if (!comm_started) {
            ROS_ERROR("Failed to start communication manager");
            return false;
        }
        
        // Start hybrid synchronizer
        bool sync_started = synchronizer_->start(
            [this](const SensorDataPacket& packet) { onSensorData(packet); }
        );
        
        if (!sync_started) {
            ROS_ERROR("Failed to start hybrid synchronizer");
            return false;
        }
        
        ROS_INFO("Robot Data Publisher Node started successfully");
        ROS_INFO("Server: %s:%d", server_host_.c_str(), server_port_);
        
        return true;
    }
    
    void stop() {
        ROS_INFO("Stopping Robot Data Publisher Node");
        
        if (synchronizer_) {
            synchronizer_->stop();
        }
        
        if (comm_manager_) {
            comm_manager_->stop();
        }
        
        // Print final statistics
        printStatistics();
    }

private:
    ros::NodeHandle nh_;
    
    std::unique_ptr<HybridSynchronizer> synchronizer_;
    std::unique_ptr<CommunicationManager> comm_manager_;
    
    // Configuration parameters
    std::string server_host_;
    int server_port_;
    int thread_pool_size_;
    bool enable_monitoring_;
    std::string log_level_;
    
    // Communication configuration
    double send_buffer_size_mb_;
    int heartbeat_interval_ms_;
    int max_reconnect_attempts_;
    bool compression_enabled_;
    double compression_threshold_;
    
    void loadParameters() {
        // Communication server parameters
        nh_.param("communication/server/host", server_host_, std::string("localhost"));
        nh_.param("communication/server/port", server_port_, 8080);
        
        // Threading parameters (kept for backward compatibility)
        nh_.param("sensor_subscriber/threading/thread_pool_size", thread_pool_size_, 4);
        
        // Performance parameters
        nh_.param("communication/performance/send_buffer_size_mb", send_buffer_size_mb_, 1.0);
        nh_.param("communication/performance/heartbeat_interval_ms", heartbeat_interval_ms_, 1000);
        nh_.param("communication/performance/max_reconnect_attempts", max_reconnect_attempts_, 10);
        
        // Compression parameters
        nh_.param("communication/compression/enabled", compression_enabled_, true);
        nh_.param("communication/compression/threshold", compression_threshold_, 0.8);
        
        // Monitoring parameters
        nh_.param("monitoring/enabled", enable_monitoring_, true);
        nh_.param("monitoring/log_level", log_level_, std::string("INFO"));
        
        ROS_INFO("Configuration loaded:");
        ROS_INFO("  Server: %s:%d", server_host_.c_str(), server_port_);
        ROS_INFO("  Threads: %d", thread_pool_size_);
        ROS_INFO("  Compression: %s", compression_enabled_ ? "enabled" : "disabled");
        ROS_INFO("  Monitoring: %s", enable_monitoring_ ? "enabled" : "disabled");
    }
    
    CommunicationManager::Config createCommunicationConfig() {
        try {
            // Try to load from YAML file first
            std::string config_path = "config/communication_config.yaml";
            
            // Check if running with roslaunch (config path relative to package)
            if (auto package_path = std::getenv("ROS_PACKAGE_PATH")) {
                config_path = std::string(package_path) + "/../robot_communication/" + config_path;
            }
            
            return CommunicationManager::Config::fromYAMLFile(config_path);
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to load YAML config: %s. Using ROS parameters as fallback.", e.what());
            
            // Fallback to ROS parameters (backward compatibility)
            CommunicationManager::Config config;
            config.server_host = server_host_;
            config.server_port = static_cast<uint16_t>(server_port_);
            config.send_buffer_size = static_cast<size_t>(send_buffer_size_mb_ * 1024 * 1024);
            config.heartbeat_interval_ms = static_cast<uint32_t>(heartbeat_interval_ms_);
            config.max_reconnect_attempts = static_cast<uint32_t>(max_reconnect_attempts_);
            config.compression_threshold = compression_threshold_;
            config.enable_adaptive_qos = true;
            
            return config;
        }
    }
    
    HybridSynchronizer::Config createSynchronizerConfig() {
        try {
            // Try to load from YAML file first
            std::string config_path = "config/communication_config.yaml";
            
            // Check if running with roslaunch (config path relative to package)
            if (auto package_path = std::getenv("ROS_PACKAGE_PATH")) {
                config_path = std::string(package_path) + "/../robot_communication/" + config_path;
            }
            
            return HybridSynchronizer::Config::fromYAMLFile(config_path);
            
        } catch (const std::exception& e) {
            ROS_WARN("Failed to load YAML sync config: %s. Using defaults.", e.what());
            
            // Fallback to default configuration
            HybridSynchronizer::Config config;
            config.queue_size = 10;
            config.max_interval_duration = 0.05;  // 50ms
            config.bandwidth_aware = true;
            config.bandwidth_threshold = 0.90;
            config.max_bandwidth_mbps = 400.0;
            config.enable_metrics = true;
            config.metrics_publish_rate = 1.0;
            
            return config;
        }
    }
    
    void onConnectionStatus(bool connected) {
        if (connected) {
            ROS_INFO("Connected to mission server at %s:%d", 
                     server_host_.c_str(), server_port_);
        } else {
            ROS_WARN("Disconnected from mission server");
        }
    }
    
    void onMissionCommand(const MissionCommand& cmd) {
        ROS_INFO("Received mission command: type=%d, id=%lu", 
                 static_cast<int>(cmd.type()), cmd.command_id());
        
        // Handle mission commands here
        switch (cmd.type()) {
            case MissionCommand::START_RECORDING:
                ROS_INFO("Starting data recording");
                break;
            case MissionCommand::STOP_RECORDING:
                ROS_INFO("Stopping data recording");
                break;
            case MissionCommand::EMERGENCY_STOP:
                ROS_WARN("Emergency stop received!");
                ros::shutdown();
                break;
            default:
                ROS_WARN("Unknown command type: %d", static_cast<int>(cmd.type()));
        }
    }
    
    void onSensorData(const SensorDataPacket& packet) {
        // Send sensor data to server
        if (comm_manager_->isConnected()) {
            if (!comm_manager_->sendSensorData(packet)) {
                ROS_WARN_THROTTLE(1.0, "Failed to send sensor data packet");
            }
        }
        
        // Print periodic stats
        static auto last_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - last_print > std::chrono::seconds(5)) {
            printStatistics();
            last_print = now;
        }
    }
    
    void printStatistics() {
        if (!synchronizer_ || !comm_manager_) return;
        
        auto sync_stats = synchronizer_->getMetrics();
        auto comm_stats = comm_manager_->getStatistics();
        
        ROS_INFO("=== Statistics ===");
        ROS_INFO("Sync: synchronized=%lu, dropped=%lu, rate=%.1f%%, latency=%.1f ms",
                 sync_stats.packets_synchronized,
                 sync_stats.packets_dropped,
                 sync_stats.sync_success_rate * 100.0,
                 sync_stats.average_sync_latency_ms);
        
        ROS_INFO("BW: current=%.1f Mbps, comm sent=%lu packets (%.1f MB), latency=%.1f ms",
                 sync_stats.current_bandwidth_mbps,
                 comm_stats.packets_sent,
                 comm_stats.bytes_sent / (1024.0 * 1024.0),
                 comm_stats.average_latency_ms);
    }
};

// Global pointer for signal handler
std::unique_ptr<RobotDataPublisherNode> g_node;

void signalHandler(int sig) {
    ROS_INFO("Received signal %d, shutting down...", sig);
    if (g_node) {
        g_node->stop();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_data_publisher");
    
    // Setup signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        g_node = std::make_unique<RobotDataPublisherNode>();
        
        if (!g_node->start()) {
            ROS_ERROR("Failed to start robot data publisher");
            return 1;
        }
        
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    if (g_node) {
        g_node->stop();
    }
    
    return 0;
}