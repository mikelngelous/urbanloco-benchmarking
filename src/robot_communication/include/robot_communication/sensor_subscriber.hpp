#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/CompressedImage.h>
#include <memory>
#include <functional>
#include <atomic>
#include <thread>
#include <queue>
#include <mutex>

#include "robot_communication/utils/thread_pool.hpp"
#include "robot_communication/utils/circular_buffer.hpp"
#include "sensor_data.pb.h"

namespace robot_communication {

/**
 * @brief High-performance sensor data subscriber with multi-threading support
 * 
 * This class subscribes to multiple ROS topics from UrbanLoco dataset and
 * provides thread-safe access to sensor data with minimal latency.
 */
class SensorSubscriber {
public:
    using SensorDataCallback = std::function<void(const SensorDataPacket&)>;
    
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param thread_pool_size Number of processing threads
     */
    explicit SensorSubscriber(ros::NodeHandle& nh, size_t thread_pool_size = 4);
    
    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~SensorSubscriber();
    
    /**
     * @brief Start subscribing to sensor topics
     * @param callback Function to call when sensor data is ready
     * @return true if successful
     */
    bool start(const SensorDataCallback& callback);
    
    /**
     * @brief Stop all subscriptions
     */
    void stop();
    
    /**
     * @brief Check if subscriber is active
     */
    bool isActive() const { return is_active_.load(); }
    
    /**
     * @brief Get performance statistics
     */
    struct Statistics {
        uint64_t messages_processed = 0;
        uint64_t messages_dropped = 0;
        double average_latency_ms = 0.0;
        double message_rate_hz = 0.0;
    };
    
    Statistics getStatistics() const;

private:
    ros::NodeHandle& nh_;
    std::unique_ptr<ThreadPool> thread_pool_;
    SensorDataCallback data_callback_;
    
    // ROS subscribers for UrbanLoco topics
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    std::vector<ros::Subscriber> camera_subs_;
    
    // Thread-safe data storage
    struct SensorBuffer {
        CircularBuffer<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer{100};
        CircularBuffer<sensor_msgs::Imu::ConstPtr> imu_buffer{500};
        CircularBuffer<sensor_msgs::NavSatFix::ConstPtr> gnss_buffer{100};
        std::array<CircularBuffer<sensor_msgs::CompressedImage::ConstPtr>, 6> camera_buffers;
        
        SensorBuffer() {
            for (auto& buffer : camera_buffers) {
                buffer.resize(200);
            }
        }
    };
    
    std::unique_ptr<SensorBuffer> sensor_buffer_;
    mutable std::mutex buffer_mutex_;
    
    // State management
    std::atomic<bool> is_active_{false};
    std::atomic<bool> should_stop_{false};
    std::unique_ptr<std::thread> fusion_thread_;
    
    // Performance tracking
    mutable std::mutex stats_mutex_;
    Statistics statistics_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Callback methods for each sensor type
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg, int camera_id);
    
    // Data fusion and synchronization
    void fusionLoop();
    bool tryFuseSensorData(SensorDataPacket& packet);
    
    // Conversion utilities
    void convertLidarData(const sensor_msgs::PointCloud2& ros_msg, LidarData& proto_msg);
    void convertImuData(const sensor_msgs::Imu& ros_msg, ImuData& proto_msg);
    void convertGnssData(const sensor_msgs::NavSatFix& ros_msg, GnssData& proto_msg);
    void convertCameraData(const sensor_msgs::CompressedImage& ros_msg, 
                          CameraData& proto_msg, int camera_id);
    
    // Utility methods
    void updateStatistics();
    ros::Time getLatestTimestamp() const;
};

} // namespace robot_communication