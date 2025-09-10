#include "robot_communication/sensor_subscriber.hpp"
#include <ros/ros.h>

namespace robot_communication {

SensorSubscriber::SensorSubscriber(ros::NodeHandle& nh, size_t thread_pool_size)
    : nh_(nh),
      thread_pool_(std::make_unique<ThreadPool>(thread_pool_size)),
      sensor_buffer_(std::make_unique<SensorBuffer>()) {
    ROS_INFO("SensorSubscriber initialized with %zu threads", thread_pool_size);
}

SensorSubscriber::~SensorSubscriber() {
    stop();
}

bool SensorSubscriber::start(const SensorDataCallback& callback) {
    if (is_active_.load()) {
        ROS_WARN("SensorSubscriber already active");
        return false;
    }
    
    data_callback_ = callback;
    
    // Subscribe to UrbanLoco topics - California format
    lidar_sub_ = nh_.subscribe("/rslidar_points", 10, 
        &SensorSubscriber::lidarCallback, this);
    
    imu_sub_ = nh_.subscribe("/imu_raw", 100, 
        &SensorSubscriber::imuCallback, this);
    
    gnss_sub_ = nh_.subscribe("/ublox_node/fix", 10, 
        &SensorSubscriber::gnssCallback, this);
    
    // Subscribe to 6 cameras for California dataset
    camera_subs_.resize(6);
    for (int i = 0; i < 6; ++i) {
        std::string topic = "/camera_array/cam" + std::to_string(i) + "/image_raw/compressed";
        camera_subs_[i] = nh_.subscribe<sensor_msgs::CompressedImage>(
            topic, 10, 
            [this, i](const sensor_msgs::CompressedImage::ConstPtr& msg) {
                this->cameraCallback(msg, i);
            });
    }
    
    // Start fusion thread
    should_stop_.store(false);
    is_active_.store(true);
    fusion_thread_ = std::make_unique<std::thread>(&SensorSubscriber::fusionLoop, this);
    start_time_ = std::chrono::steady_clock::now();
    
    ROS_INFO("SensorSubscriber started successfully");
    return true;
}

void SensorSubscriber::stop() {
    if (!is_active_.load()) {
        return;
    }
    
    should_stop_.store(true);
    is_active_.store(false);
    
    if (fusion_thread_ && fusion_thread_->joinable()) {
        fusion_thread_->join();
    }
    
    // Shutdown all subscribers
    lidar_sub_.shutdown();
    imu_sub_.shutdown();
    gnss_sub_.shutdown();
    
    for (auto& sub : camera_subs_) {
        sub.shutdown();
    }
    camera_subs_.clear();
    
    ROS_INFO("SensorSubscriber stopped");
}

void SensorSubscriber::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!sensor_buffer_->lidar_buffer.push(msg)) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.messages_dropped++;
    }
}

void SensorSubscriber::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!sensor_buffer_->imu_buffer.push(msg)) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.messages_dropped++;
    }
}

void SensorSubscriber::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!sensor_buffer_->gnss_buffer.push(msg)) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        statistics_.messages_dropped++;
    }
}

void SensorSubscriber::cameraCallback(const sensor_msgs::CompressedImage::ConstPtr& msg, int camera_id) {
    if (camera_id >= 0 && camera_id < 6) {
        if (!sensor_buffer_->camera_buffers[camera_id].push(msg)) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            statistics_.messages_dropped++;
        }
    }
}

void SensorSubscriber::fusionLoop() {
    const auto fusion_rate = std::chrono::milliseconds(10); // 100Hz
    
    while (!should_stop_.load()) {
        SensorDataPacket packet;
        
        if (tryFuseSensorData(packet)) {
            if (data_callback_) {
                data_callback_(packet);
            }
            
            std::lock_guard<std::mutex> lock(stats_mutex_);
            statistics_.messages_processed++;
        }
        
        std::this_thread::sleep_for(fusion_rate);
    }
}

bool SensorSubscriber::tryFuseSensorData(SensorDataPacket& packet) {
    // For now, return basic packet structure
    // Full implementation would synchronize sensor data by timestamp
    
    packet.set_sequence_id(statistics_.messages_processed);
    packet.set_frame_id("robot_frame");
    
    auto timestamp = packet.mutable_timestamp();
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    timestamp->set_seconds(seconds.count());
    timestamp->set_nanos(static_cast<int32_t>(nanos.count()));
    
    return true; // Basic implementation always succeeds
}

SensorSubscriber::Statistics SensorSubscriber::getStatistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return statistics_;
}

} // namespace robot_communication