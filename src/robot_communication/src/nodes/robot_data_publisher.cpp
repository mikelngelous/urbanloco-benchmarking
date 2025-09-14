#include <ros/ros.h>
#include <signal.h>
#include "robot_communication/hybrid_synchronizer.hpp"
#include "robot_communication/communication_manager.hpp"
#include "robot_communication/data_transfer_queue.hpp"
#include "robot_communication/camera_timestamp_rectifier.hpp"

using namespace robot_communication;

class RobotDataPublisherNode {
public:
    RobotDataPublisherNode() : nh_("~") {
        ROS_INFO("Starting Robot Data Publisher Node");
        ROS_INFO("ROS Master URI: %s", ros::master::getURI().c_str());
        
        // Load parameters from YAML configuration
        ROS_INFO("Loading parameters...");
        loadParameters();
        ROS_INFO("Parameters loaded successfully");
        
        // Initialize components
        ROS_INFO("Creating HybridSynchronizer configuration...");
        HybridSynchronizer::Config sync_config = createSynchronizerConfig();
        ROS_INFO("Initializing HybridSynchronizer...");
        synchronizer_ = std::make_unique<HybridSynchronizer>(nh_, sync_config);
        ROS_INFO("HybridSynchronizer created successfully");
        
        ROS_INFO("Creating CommunicationManager configuration...");
        CommunicationManager::Config comm_config = createCommunicationConfig();
        ROS_INFO("Initializing CommunicationManager...");
        comm_manager_ = std::make_unique<CommunicationManager>(comm_config);
        ROS_INFO("CommunicationManager created successfully");
        
        // Initialize data transfer queue with decoupling capabilities
        ROS_INFO("Initializing DataTransferQueue for producer-consumer decoupling...");
        DataTransferQueue::Config queue_config;
        queue_config.max_queue_size = 500;              // Buffer up to 500 packets
        queue_config.drop_oldest_on_full = true;        // Drop oldest when full
        queue_config.timeout_ms = std::chrono::milliseconds(100);
        queue_config.enable_metrics = true;
        data_queue_ = std::make_unique<DataTransferQueue>(queue_config);
        ROS_INFO("DataTransferQueue created successfully");
        
        // Initialize camera timestamp rectifier for real-time conversion
        ROS_INFO("Initializing CameraTimestampRectifier for real-time timestamps...");
        camera_rectifier_ = std::make_unique<CameraTimestampRectifier>(
            nh_, 
            "/camera_array/cam0/image_raw/compressed",           // Raw input with zero timestamps
            "/camera_array/cam0/image_raw/compressed/rectified"  // Output with real-time timestamps
        );
        ROS_INFO("CameraTimestampRectifier created successfully");
        
        ROS_INFO("RobotDataPublisherNode constructor completed");
    }
    
    bool start() {
        ROS_INFO("Starting robot data publisher components...");
        
        // Start communication manager
        ROS_INFO("Starting CommunicationManager...");
        try {
            bool comm_started = comm_manager_->start(
                [this](bool connected) { onConnectionStatus(connected); },
                [this](const MissionCommand& cmd) { onMissionCommand(cmd); }
            );
            
            if (!comm_started) {
                ROS_ERROR("Failed to start communication manager");
                return false;
            }
            ROS_INFO("CommunicationManager started successfully");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception starting CommunicationManager: %s", e.what());
            return false;
        }
        
        // Start camera timestamp rectifier for real-time conversion
        ROS_INFO("Starting CameraTimestampRectifier...");
        camera_rectifier_->start();
        ROS_INFO("CameraTimestampRectifier started - converting camera timestamps to real-time");
        
        // Start network worker thread (Consumer)
        ROS_INFO("Starting network worker thread for decoupled communication...");
        network_thread_ = std::make_unique<std::thread>(&RobotDataPublisherNode::networkWorkerThread, this);
        ROS_INFO("Network worker thread started successfully");
        
        // Start hybrid synchronizer (Producer) 
        ROS_INFO("Starting HybridSynchronizer...");
        try {
            bool sync_started = synchronizer_->start(
                [this](const SensorDataPacket& packet) { onSensorData(packet); }
            );
            
            if (!sync_started) {
                ROS_ERROR("Failed to start hybrid synchronizer");
                return false;
            }
            ROS_INFO("HybridSynchronizer started successfully");
        } catch (const std::exception& e) {
            ROS_ERROR("Exception starting HybridSynchronizer: %s", e.what());
            return false;
        }
        
        ROS_INFO("Robot Data Publisher Node started successfully with Producer-Consumer architecture");
        ROS_INFO("Server: %s:%d", server_host_.c_str(), server_port_);
        
        return true;
    }
    
    void stop() {
        ROS_INFO("Stopping Robot Data Publisher Node with Producer-Consumer architecture");
        
        if (synchronizer_) {
            ROS_INFO("Stopping HybridSynchronizer (Producer)...");
            synchronizer_->stop();
        }
        
        if (camera_rectifier_) {
            ROS_INFO("Stopping CameraTimestampRectifier...");
            camera_rectifier_->stop();
        }
        
        if (data_queue_) {
            ROS_INFO("Shutting down DataTransferQueue...");
            data_queue_->shutdown();
        }
        
        if (network_thread_ && network_thread_->joinable()) {
            ROS_INFO("Stopping network worker thread (Consumer)...");
            network_thread_->join();
            ROS_INFO("Network worker thread stopped");
        }
        
        if (comm_manager_) {
            ROS_INFO("Stopping CommunicationManager...");
            comm_manager_->stop();
        }
        
        // Print final statistics
        printStatistics();
    }

private:
    ros::NodeHandle nh_;
    
    std::unique_ptr<HybridSynchronizer> synchronizer_;
    std::unique_ptr<CommunicationManager> comm_manager_;
    std::unique_ptr<DataTransferQueue> data_queue_;
    std::unique_ptr<std::thread> network_thread_;
    std::unique_ptr<CameraTimestampRectifier> camera_rectifier_;
    
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
        // Load configuration from ROS parameters (loaded via rosparam load)
        ROS_INFO("Loading HybridSynchronizer config from ROS parameters...");
        
        HybridSynchronizer::Config config;
        
        // Synchronization parameters from synchronizer/sync_params
        nh_.param("synchronizer/sync_params/queue_size", config.queue_size, 10);
        nh_.param("synchronizer/sync_params/max_time_difference", config.max_interval_duration, 0.1);  // 100ms from YAML
        
        // Bandwidth parameters from synchronizer/bandwidth
        nh_.param("synchronizer/bandwidth/target_mbps", config.max_bandwidth_mbps, 422.5);  // From YAML
        config.bandwidth_aware = true;
        config.bandwidth_threshold = 0.8;  // Drop threshold from YAML
        
        // Enable metrics from monitoring section
        bool enable_metrics = false;
        nh_.param("monitoring/enable_metrics", enable_metrics, true);
        config.enable_metrics = enable_metrics;
        
        double log_interval = 1.0;
        nh_.param("monitoring/log_interval", log_interval, 1.0);
        config.metrics_publish_rate = log_interval;
        
        ROS_INFO("HybridSynchronizer config loaded from ROS parameters:");
        ROS_INFO("  queue_size: %d", config.queue_size);
        ROS_INFO("  max_time_difference: %.3f s", config.max_interval_duration);
        ROS_INFO("  max_bandwidth_mbps: %.1f", config.max_bandwidth_mbps);
        ROS_INFO("  bandwidth_aware: %s", config.bandwidth_aware ? "enabled" : "disabled");
        ROS_INFO("  enable_metrics: %s", config.enable_metrics ? "enabled" : "disabled");
        
        return config;
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
        // PRODUCER: Non-blocking enqueue to transfer queue
        if (!data_queue_->enqueue(packet)) {
            ROS_WARN_THROTTLE(5.0, "Producer queue full, packet dropped");
        }

        // Print periodic stats
        static auto last_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - last_print > std::chrono::seconds(10)) {
            printStatistics();
            last_print = now;
        }
    }
    
    void printStatistics() {
        if (!synchronizer_ || !comm_manager_ || !data_queue_) return;
        
        auto sync_stats = synchronizer_->getMetrics();
        auto comm_stats = comm_manager_->getStatistics();
        auto queue_metrics = data_queue_->getMetrics();
        
        ROS_INFO("=== Producer-Consumer Statistics ===");
        ROS_INFO("Sync: synchronized=%lu, dropped=%lu, rate=%.1f%%, latency=%.1f ms",
                 sync_stats.packets_synchronized,
                 sync_stats.packets_dropped,
                 sync_stats.sync_success_rate * 100.0,
                 sync_stats.average_sync_latency_ms);
        
        ROS_INFO("Queue: enqueued=%lu, dequeued=%lu, dropped=%lu, current_size=%zu, peak_size=%zu",
                 queue_metrics.packets_enqueued.load(),
                 queue_metrics.packets_dequeued.load(),
                 queue_metrics.packets_dropped.load(),
                 queue_metrics.current_size.load(),
                 queue_metrics.peak_size.load());
        
        ROS_INFO("Queue Perf: drop_rate=%.2f%%, throughput=%.1f Hz",
                 queue_metrics.getDropRate() * 100.0,
                 queue_metrics.getThroughputHz());
        
        ROS_INFO("BW: current=%.1f Mbps, comm sent=%lu packets (%.1f MB), latency=%.1f ms",
                 sync_stats.current_bandwidth_mbps,
                 comm_stats.packets_sent,
                 comm_stats.bytes_sent / (1024.0 * 1024.0),
                 comm_stats.average_latency_ms);
    }

    // CONSUMER: Network worker thread that processes data from the queue
    void networkWorkerThread() {
        ROS_DEBUG("Network worker thread started");
        SensorDataPacket packet;
        uint32_t processed_count = 0;

        while (ros::ok()) {
            // Blocking dequeue with timeout - this isolates network operations
            if (data_queue_->dequeue(packet, std::chrono::milliseconds(100))) {
                processed_count++;

                // Network operations isolated in consumer thread
                if (comm_manager_->isConnected()) {
                    if (!comm_manager_->sendSensorData(packet)) {
                        ROS_WARN_THROTTLE(5.0, "Failed to send sensor data via TCP");
                    }
                }
            }
        }

        ROS_INFO("Network worker thread exiting, processed %u packets", processed_count);
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