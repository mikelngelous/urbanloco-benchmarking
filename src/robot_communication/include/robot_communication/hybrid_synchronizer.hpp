#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CompressedImage.h>

#include <yaml-cpp/yaml.h>
#include <functional>
#include <atomic>
#include <chrono>

#include "sensor_data.pb.h"

namespace robot_communication {

/**
 * @brief Hybrid synchronizer using message_filters::ApproximateTime + bandwidth management
 * 
 * Combines ROS-proven ApproximateTime synchronization with intelligent bandwidth
 * management for handling 422.5 Mbps UrbanLoco dataset requirements.
 */
class HybridSynchronizer {
public:
    using SyncCallback = std::function<void(const SensorDataPacket&)>;
    
    // ApproximateTime policy for all sensors including 6 cameras
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,     // LiDAR
        sensor_msgs::Imu,             // IMU
        sensor_msgs::NavSatFix,       // GNSS
        sensor_msgs::CompressedImage, // Camera cam0
        sensor_msgs::CompressedImage, // Camera cam1
        sensor_msgs::CompressedImage, // Camera cam2
        sensor_msgs::CompressedImage, // Camera cam3
        sensor_msgs::CompressedImage, // Camera cam4
        sensor_msgs::CompressedImage  // Camera cam5
    > MultiCameraPolicy;
    
    struct Config {
        // message_filters configuration
        int queue_size = 10;
        double max_interval_duration = 0.05;  // 50ms max window
        double age_penalty = 0.1;              // Prefer recent messages
        
        // Bandwidth management
        bool bandwidth_aware = true;
        double bandwidth_threshold = 0.90;     // 90% of max bandwidth
        double max_bandwidth_mbps = 400.0;     // Actual network capacity
        
        // Priority configuration
        enum Priority { CRITICAL = 0, HIGH = 1, MEDIUM = 2, LOW = 3 };
        struct SensorPriorities {
            Priority lidar = CRITICAL;
            Priority imu = CRITICAL;
            Priority gnss = MEDIUM;
            Priority camera_front = HIGH;    // cam0
            Priority camera_left = MEDIUM;   // cam1  
            Priority camera_right = MEDIUM;  // cam2
            Priority camera_rear = LOW;      // cam3
            Priority camera_up = LOW;        // cam4
            Priority camera_down = LOW;      // cam5
        } priorities;
        
        // Performance monitoring
        bool enable_metrics = true;
        double metrics_publish_rate = 1.0;    // Hz
        
        // Load from YAML
        static Config fromYAMLFile(const std::string& file_path);
        static Config fromYAML(const YAML::Node& node);
    };
    
    /**
     * @brief Constructor
     */
    explicit HybridSynchronizer(ros::NodeHandle& nh, const Config& config);
    
    /**
     * @brief Destructor
     */
    ~HybridSynchronizer();
    
    /**
     * @brief Start synchronization
     */
    bool start(const SyncCallback& callback);
    
    /**
     * @brief Stop synchronization
     */
    void stop();
    
    /**
     * @brief Check if synchronizer is active
     */
    bool isActive() const { return is_active_.load(); }
    
    /**
     * @brief Get performance metrics
     */
    struct Metrics {
        uint64_t packets_synchronized = 0;
        uint64_t packets_dropped = 0;
        double average_sync_latency_ms = 0.0;
        double current_bandwidth_mbps = 0.0;
        double sync_success_rate = 0.0;
        std::chrono::steady_clock::time_point last_update;
    };
    
    Metrics getMetrics() const;

private:
    ros::NodeHandle& nh_;
    Config config_;
    SyncCallback sync_callback_;
    
    // message_filters subscribers for all sensors
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gnss_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam0_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam1_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam2_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam3_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam4_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> cam5_sub_;
    
    // Debug subscribers for individual message tracking
    ros::Subscriber debug_lidar_sub_;
    ros::Subscriber debug_imu_sub_;
    ros::Subscriber debug_gnss_sub_;
    ros::Subscriber debug_cam0_sub_;
    
    // Debug counters
    std::atomic<uint32_t> debug_lidar_count_{0};
    std::atomic<uint32_t> debug_imu_count_{0};
    std::atomic<uint32_t> debug_gnss_count_{0};
    std::atomic<uint32_t> debug_cam0_count_{0};
    
    // Multi-camera synchronizer
    std::unique_ptr<message_filters::Synchronizer<MultiCameraPolicy>> sync_;
    
    // State management
    std::atomic<bool> is_active_{false};
    mutable std::mutex metrics_mutex_;
    Metrics metrics_;
    
    // Bandwidth monitoring
    class BandwidthMonitor {
    public:
        explicit BandwidthMonitor(const Config& config);
        
        bool allowTransmission(size_t estimated_packet_size);
        void recordTransmission(size_t actual_size);
        double getCurrentUsage() const;
        void updateConfig(const Config& config);
        
    private:
        Config config_;
        mutable std::atomic<double> current_bandwidth_mbps_{0.0};
        mutable std::chrono::steady_clock::time_point last_measurement_;
        mutable std::atomic<uint64_t> bytes_transmitted_{0};
        mutable std::mutex bandwidth_mutex_;
    };
    
    std::unique_ptr<BandwidthMonitor> bandwidth_monitor_;
    
    // Core synchronization callback with all 6 cameras
    void synchronizedCallback(
        const sensor_msgs::PointCloud2::ConstPtr& lidar,
        const sensor_msgs::Imu::ConstPtr& imu,
        const sensor_msgs::NavSatFix::ConstPtr& gnss,
        const sensor_msgs::CompressedImage::ConstPtr& cam0,
        const sensor_msgs::CompressedImage::ConstPtr& cam1,
        const sensor_msgs::CompressedImage::ConstPtr& cam2,
        const sensor_msgs::CompressedImage::ConstPtr& cam3,
        const sensor_msgs::CompressedImage::ConstPtr& cam4,
        const sensor_msgs::CompressedImage::ConstPtr& cam5
    );
    
    // Debug callbacks for individual message tracking
    void debugLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void debugImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void debugGnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void debugCam0Callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    
    // Bandwidth-aware processing  
    bool shouldDropPacket(const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras);
    std::vector<sensor_msgs::CompressedImage::ConstPtr> applyPriorityDropping(
        const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras);
    
    // Packet building (overloaded versions)
    SensorDataPacket buildSensorPacket(
        const sensor_msgs::PointCloud2::ConstPtr& lidar,
        const sensor_msgs::Imu::ConstPtr& imu,
        const sensor_msgs::NavSatFix::ConstPtr& gnss
    );
    
    SensorDataPacket buildSensorPacket(
        const sensor_msgs::PointCloud2::ConstPtr& lidar,
        const sensor_msgs::Imu::ConstPtr& imu,
        const sensor_msgs::NavSatFix::ConstPtr& gnss,
        const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras
    );
    
    // Conversion utilities  
    void convertLidarData(const sensor_msgs::PointCloud2& ros_msg, LidarData& proto_msg) const;
    void convertImuData(const sensor_msgs::Imu& ros_msg, ImuData& proto_msg) const;
    void convertGnssData(const sensor_msgs::NavSatFix& ros_msg, GnssData& proto_msg) const;
    void convertCameraData(const sensor_msgs::CompressedImage& ros_msg, CameraData& proto_msg, int camera_id) const;
    
    // Metrics
    void updateMetrics();
    size_t estimatePacketSize(
        const sensor_msgs::PointCloud2::ConstPtr& lidar,
        const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras
    );
};

} // namespace robot_communication