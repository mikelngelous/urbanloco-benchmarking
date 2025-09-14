#include "robot_communication/hybrid_synchronizer.hpp"
#include <boost/function.hpp>
#include <boost/function.hpp>

using boost::placeholders::_1;
using boost::placeholders::_2;
using boost::placeholders::_3;
using boost::placeholders::_4;
using boost::placeholders::_5;
using boost::placeholders::_6;
using boost::placeholders::_7;
using boost::placeholders::_8;
using boost::placeholders::_9;
#include <ros/ros.h>
#include <algorithm>

namespace robot_communication {

// BandwidthMonitor Implementation
HybridSynchronizer::BandwidthMonitor::BandwidthMonitor(const Config& config) 
    : config_(config), last_measurement_(std::chrono::steady_clock::now()) {
}

bool HybridSynchronizer::BandwidthMonitor::allowTransmission(size_t estimated_packet_size) {
    if (!config_.bandwidth_aware) {
        return true;
    }
    
    double current_usage = getCurrentUsage();
    double estimated_additional_mbps = (estimated_packet_size * 8.0) / (1024.0 * 1024.0); // Convert to Mbps
    
    return (current_usage + estimated_additional_mbps) < (config_.max_bandwidth_mbps * config_.bandwidth_threshold);
}

void HybridSynchronizer::BandwidthMonitor::recordTransmission(size_t actual_size) {
    bytes_transmitted_.fetch_add(actual_size, std::memory_order_relaxed);
}

double HybridSynchronizer::BandwidthMonitor::getCurrentUsage() const {
    std::lock_guard<std::mutex> lock(bandwidth_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_measurement_);
    
    if (duration.count() > 1000) { // Update every second
        double seconds = duration.count() / 1000.0;
        uint64_t bytes = bytes_transmitted_.exchange(0, std::memory_order_relaxed);
        current_bandwidth_mbps_.store((bytes * 8.0) / (1024.0 * 1024.0 * seconds), std::memory_order_relaxed);
        last_measurement_ = now;
    }
    
    return current_bandwidth_mbps_.load(std::memory_order_relaxed);
}

void HybridSynchronizer::BandwidthMonitor::updateConfig(const Config& config) {
    config_ = config;
}

// Config YAML loading
HybridSynchronizer::Config HybridSynchronizer::Config::fromYAMLFile(const std::string& file_path) {
    YAML::Node yaml_config = YAML::LoadFile(file_path);
    return fromYAML(yaml_config["synchronization"]);
}

HybridSynchronizer::Config HybridSynchronizer::Config::fromYAML(const YAML::Node& node) {
    Config config;
    
    // message_filters settings
    if (node["message_filters"]) {
        auto mf = node["message_filters"];
        config.queue_size = mf["queue_size"].as<int>(config.queue_size);
        config.max_interval_duration = mf["max_interval_duration"].as<double>(config.max_interval_duration);
        config.age_penalty = mf["age_penalty"].as<double>(config.age_penalty);
    }
    
    // Bandwidth management
    if (node["bandwidth_management"]) {
        auto bw = node["bandwidth_management"];
        config.bandwidth_aware = bw["enabled"].as<bool>(config.bandwidth_aware);
        config.bandwidth_threshold = bw["threshold"].as<double>(config.bandwidth_threshold);
        config.max_bandwidth_mbps = bw["max_bandwidth_mbps"].as<double>(config.max_bandwidth_mbps);
    }
    
    // Sensor priorities
    if (node["sensor_priorities"]) {
        auto priorities = node["sensor_priorities"];
        
        auto parsePriority = [](const std::string& priority_str) -> Priority {
            if (priority_str == "critical") return CRITICAL;
            if (priority_str == "high") return HIGH;
            if (priority_str == "medium") return MEDIUM;
            if (priority_str == "low") return LOW;
            return MEDIUM;
        };
        
        if (priorities["lidar"]) config.priorities.lidar = parsePriority(priorities["lidar"].as<std::string>());
        if (priorities["imu"]) config.priorities.imu = parsePriority(priorities["imu"].as<std::string>());
        if (priorities["gnss"]) config.priorities.gnss = parsePriority(priorities["gnss"].as<std::string>());
        if (priorities["camera_front"]) config.priorities.camera_front = parsePriority(priorities["camera_front"].as<std::string>());
        if (priorities["camera_left"]) config.priorities.camera_left = parsePriority(priorities["camera_left"].as<std::string>());
        if (priorities["camera_right"]) config.priorities.camera_right = parsePriority(priorities["camera_right"].as<std::string>());
        if (priorities["camera_rear"]) config.priorities.camera_rear = parsePriority(priorities["camera_rear"].as<std::string>());
        if (priorities["camera_up"]) config.priorities.camera_up = parsePriority(priorities["camera_up"].as<std::string>());
        if (priorities["camera_down"]) config.priorities.camera_down = parsePriority(priorities["camera_down"].as<std::string>());
    }
    
    return config;
}

// HybridSynchronizer Implementation
HybridSynchronizer::HybridSynchronizer(ros::NodeHandle& nh, const Config& config)
    : nh_(nh), config_(config) {
    
    bandwidth_monitor_ = std::make_unique<BandwidthMonitor>(config_);
    
    ROS_INFO("HybridSynchronizer initialized with:");
    ROS_INFO("  Queue size: %d", config_.queue_size);
    ROS_INFO("  Max interval: %.3f s", config_.max_interval_duration);
    ROS_INFO("  Bandwidth aware: %s", config_.bandwidth_aware ? "enabled" : "disabled");
    ROS_INFO("  Max bandwidth: %.1f Mbps", config_.max_bandwidth_mbps);
}

HybridSynchronizer::~HybridSynchronizer() {
    stop();
}

bool HybridSynchronizer::start(const SyncCallback& callback) {
    if (is_active_.load()) {
        ROS_WARN("HybridSynchronizer already active");
        return false;
    }
    
    sync_callback_ = callback;
    
    // Get topic names from ROS parameters (configurable for dataset vs real sensors)
    // Use global nodehandle to access parameters loaded via rosparam load
    ros::NodeHandle global_nh;
    std::string lidar_topic, imu_topic, gnss_topic, camera0_topic;
    std::string camera1_topic, camera2_topic, camera3_topic, camera4_topic, camera5_topic;
    global_nh.param("synchronizer/topics/lidar", lidar_topic, std::string("/sensors/lidar_realtime"));
    global_nh.param("synchronizer/topics/imu", imu_topic, std::string("/sensors/imu_realtime"));
    global_nh.param("synchronizer/topics/gnss", gnss_topic, std::string("/sensors/gnss_realtime"));
    global_nh.param("synchronizer/topics/camera0", camera0_topic, std::string("/sensors/camera0_realtime"));
    global_nh.param("synchronizer/topics/camera1", camera1_topic, std::string("/sensors/camera1_realtime"));
    global_nh.param("synchronizer/topics/camera2", camera2_topic, std::string("/sensors/camera2_realtime"));
    global_nh.param("synchronizer/topics/camera3", camera3_topic, std::string("/sensors/camera3_realtime"));
    global_nh.param("synchronizer/topics/camera4", camera4_topic, std::string("/sensors/camera4_realtime"));
    global_nh.param("synchronizer/topics/camera5", camera5_topic, std::string("/sensors/camera5_realtime"));
    
    ROS_INFO("Using topic configuration:");
    ROS_INFO("  LiDAR: %s", lidar_topic.c_str());
    ROS_INFO("  IMU: %s", imu_topic.c_str());
    ROS_INFO("  GNSS: %s", gnss_topic.c_str());
    ROS_INFO("  Camera0: %s", camera0_topic.c_str());
    ROS_INFO("  Additional cameras: cam1-cam5 configured for future expansion");
    
    // Initialize message_filters subscribers for all sensors
    lidar_sub_.subscribe(nh_, lidar_topic, config_.queue_size);
    imu_sub_.subscribe(nh_, imu_topic, config_.queue_size);
    gnss_sub_.subscribe(nh_, gnss_topic, config_.queue_size);
    cam0_sub_.subscribe(nh_, camera0_topic, config_.queue_size);
    cam1_sub_.subscribe(nh_, camera1_topic, config_.queue_size);
    cam2_sub_.subscribe(nh_, camera2_topic, config_.queue_size);
    cam3_sub_.subscribe(nh_, camera3_topic, config_.queue_size);
    cam4_sub_.subscribe(nh_, camera4_topic, config_.queue_size);
    cam5_sub_.subscribe(nh_, camera5_topic, config_.queue_size);
    
    // DEBUG: Initialize individual tracking subscribers  
    debug_lidar_sub_ = nh_.subscribe(lidar_topic, 1, &HybridSynchronizer::debugLidarCallback, this);
    debug_imu_sub_ = nh_.subscribe(imu_topic, 1, &HybridSynchronizer::debugImuCallback, this);
    debug_gnss_sub_ = nh_.subscribe(gnss_topic, 1, &HybridSynchronizer::debugGnssCallback, this);
    debug_cam0_sub_ = nh_.subscribe(camera0_topic, 1, &HybridSynchronizer::debugCam0Callback, this);
    
    // Create Multi-camera synchronizer (9 sensors total)
    sync_ = std::make_unique<message_filters::Synchronizer<MultiCameraPolicy>>(
        MultiCameraPolicy(config_.queue_size),
        lidar_sub_, imu_sub_, gnss_sub_,
        cam0_sub_, cam1_sub_, cam2_sub_, cam3_sub_, cam4_sub_, cam5_sub_
    );
    
    // Configure synchronizer - VERY RELAXED for real dataset with 2s LiDAR intervals  
    sync_->setMaxIntervalDuration(ros::Duration(2.5));  // 2.5s tolerance for LiDAR timing
    sync_->setAgePenalty(0.01); // Very low penalty for large time differences
    
    // Register callback using wrapper function - boost::function supports 9 parameters
    sync_->registerCallback(boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&,
                                                 const sensor_msgs::Imu::ConstPtr&,
                                                 const sensor_msgs::NavSatFix::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&,
                                                 const sensor_msgs::CompressedImage::ConstPtr&)>(
        [this](const sensor_msgs::PointCloud2::ConstPtr& lidar,
               const sensor_msgs::Imu::ConstPtr& imu,
               const sensor_msgs::NavSatFix::ConstPtr& gnss,
               const sensor_msgs::CompressedImage::ConstPtr& cam0,
               const sensor_msgs::CompressedImage::ConstPtr& cam1,
               const sensor_msgs::CompressedImage::ConstPtr& cam2,
               const sensor_msgs::CompressedImage::ConstPtr& cam3,
               const sensor_msgs::CompressedImage::ConstPtr& cam4,
               const sensor_msgs::CompressedImage::ConstPtr& cam5) {
            this->synchronizedCallback(lidar, imu, gnss, cam0, cam1, cam2, cam3, cam4, cam5);
        }));
    
    is_active_.store(true);
    
    ROS_INFO("HybridSynchronizer started successfully");
    ROS_INFO("Subscribed to synchronized topics:");
    ROS_INFO("  LiDAR: %s", lidar_topic.c_str());
    ROS_INFO("  IMU: %s", imu_topic.c_str()); 
    ROS_INFO("  GNSS: %s", gnss_topic.c_str());
    ROS_INFO("  Camera: %s (with real-time timestamps)", camera0_topic.c_str());
    
    return true;
}

void HybridSynchronizer::stop() {
    if (!is_active_.load()) {
        return;
    }
    
    is_active_.store(false);
    
    // Reset synchronizer (automatically unsubscribes)
    sync_.reset();
    
    ROS_INFO("HybridSynchronizer stopped");
}

void HybridSynchronizer::synchronizedCallback(
    const sensor_msgs::PointCloud2::ConstPtr& lidar,
    const sensor_msgs::Imu::ConstPtr& imu,
    const sensor_msgs::NavSatFix::ConstPtr& gnss,
    const sensor_msgs::CompressedImage::ConstPtr& cam0,
    const sensor_msgs::CompressedImage::ConstPtr& cam1,
    const sensor_msgs::CompressedImage::ConstPtr& cam2,
    const sensor_msgs::CompressedImage::ConstPtr& cam3,
    const sensor_msgs::CompressedImage::ConstPtr& cam4,
    const sensor_msgs::CompressedImage::ConstPtr& cam5) {

    size_t total_camera_bytes = cam0->data.size() + cam1->data.size() + cam2->data.size() +
                               cam3->data.size() + cam4->data.size() + cam5->data.size();

    ROS_INFO_THROTTLE(2.0, "SYNCHRONIZED 9 SENSORS! LiDAR: %u pts, IMU(%.2f,%.2f,%.2f), GNSS: %.6f, 6 Cams: %zu KB total",
                     lidar->width * lidar->height,
                     imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z,
                     gnss->latitude,
                     total_camera_bytes / 1024);
    
    if (!sync_callback_) {
        ROS_WARN_THROTTLE(5.0, "No sync callback registered");
        return;
    }
    
    // Build synchronized packet (including camera with real-time timestamp)
    auto start_time = std::chrono::steady_clock::now();
    
    // Create vector with all 6 cameras
    std::vector<sensor_msgs::CompressedImage::ConstPtr> cameras;
    cameras.push_back(cam0);
    cameras.push_back(cam1);
    cameras.push_back(cam2);
    cameras.push_back(cam3);
    cameras.push_back(cam4);
    cameras.push_back(cam5);

    SensorDataPacket packet = buildSensorPacket(lidar, imu, gnss, cameras);

    auto end_time = std::chrono::steady_clock::now();
    double processing_latency = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Record bandwidth usage (estimated size for 9 sensors including 6 cameras)
    size_t estimated_size = lidar->data.size() + sizeof(sensor_msgs::Imu) + sizeof(sensor_msgs::NavSatFix) + total_camera_bytes;
    bandwidth_monitor_->recordTransmission(estimated_size);
    
    // Update metrics
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.packets_synchronized++;
        metrics_.average_sync_latency_ms = (metrics_.average_sync_latency_ms * 0.9) + (processing_latency * 0.1);
        metrics_.current_bandwidth_mbps = bandwidth_monitor_->getCurrentUsage();
        metrics_.last_update = std::chrono::steady_clock::now();
        
        if (metrics_.packets_synchronized + metrics_.packets_dropped > 0) {
            metrics_.sync_success_rate = static_cast<double>(metrics_.packets_synchronized) / 
                (metrics_.packets_synchronized + metrics_.packets_dropped);
        }
    }
    
    // Forward to callback
    sync_callback_(packet);
    
    ROS_DEBUG("Synchronized packet processed: latency=%.2fms, size=%zuB", processing_latency, estimated_size);
}

std::vector<sensor_msgs::CompressedImage::ConstPtr> HybridSynchronizer::applyPriorityDropping(
    const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras) {
    
    std::vector<std::pair<sensor_msgs::CompressedImage::ConstPtr, Config::Priority>> camera_priorities = {
        {cameras[0], config_.priorities.camera_front},  // cam0
        {cameras[1], config_.priorities.camera_left},   // cam1  
        {cameras[2], config_.priorities.camera_right},  // cam2
        {cameras[3], config_.priorities.camera_rear},   // cam3
        {cameras[4], config_.priorities.camera_up},     // cam4
        {cameras[5], config_.priorities.camera_down}    // cam5
    };
    
    // Sort by priority (CRITICAL=0, HIGH=1, MEDIUM=2, LOW=3)
    std::sort(camera_priorities.begin(), camera_priorities.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second; // Lower number = higher priority
        });
    
    // Keep high-priority cameras, drop low-priority ones
    std::vector<sensor_msgs::CompressedImage::ConstPtr> filtered_cameras;
    
    // Always keep CRITICAL and HIGH priority cameras
    for (const auto& [camera, priority] : camera_priorities) {
        if (priority <= Config::HIGH) {
            filtered_cameras.push_back(camera);
        } else if (filtered_cameras.size() < 3) {
            // Keep at least 3 cameras total (including some MEDIUM priority)
            filtered_cameras.push_back(camera);
        }
    }
    
    ROS_DEBUG("Priority dropping: kept %zu/%zu cameras", filtered_cameras.size(), cameras.size());
    
    return filtered_cameras;
}

// Simplified version for 3 synchronized sensors only
SensorDataPacket HybridSynchronizer::buildSensorPacket(
    const sensor_msgs::PointCloud2::ConstPtr& lidar,
    const sensor_msgs::Imu::ConstPtr& imu,
    const sensor_msgs::NavSatFix::ConstPtr& gnss) {
    
    SensorDataPacket packet;
    
    // Set metadata
    static uint64_t sequence_id = 0;
    packet.set_sequence_id(++sequence_id);
    
    // Set timestamp (use LiDAR timestamp as reference)
    auto timestamp = packet.mutable_timestamp();
    timestamp->set_seconds(lidar->header.stamp.sec);
    timestamp->set_nanos(lidar->header.stamp.nsec);
    
    // Convert sensor data
    if (lidar) {
        auto proto_lidar = packet.mutable_lidar_data();
        convertLidarData(*lidar, *proto_lidar);
    }
    
    if (imu) {
        auto proto_imu = packet.mutable_imu_data();
        convertImuData(*imu, *proto_imu);
    }
    
    if (gnss) {
        auto proto_gnss = packet.mutable_gnss_data();
        convertGnssData(*gnss, *proto_gnss);
    }
    
    return packet;
}

// Full version with cameras (kept for future use)
SensorDataPacket HybridSynchronizer::buildSensorPacket(
    const sensor_msgs::PointCloud2::ConstPtr& lidar,
    const sensor_msgs::Imu::ConstPtr& imu,
    const sensor_msgs::NavSatFix::ConstPtr& gnss,
    const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras) {
    
    SensorDataPacket packet;
    
    // Set metadata
    static uint64_t sequence_id = 0;
    packet.set_sequence_id(++sequence_id);
    packet.set_frame_id("robot_frame");
    
    // Use LiDAR timestamp as reference (most stable)
    auto timestamp = packet.mutable_timestamp();
    auto lidar_time = lidar->header.stamp;
    timestamp->set_seconds(lidar_time.sec);
    timestamp->set_nanos(lidar_time.nsec);
    
    // Convert sensor data
    if (lidar) {
        auto proto_lidar = packet.mutable_lidar_data();
        convertLidarData(*lidar, *proto_lidar);
    }
    
    if (imu) {
        auto proto_imu = packet.mutable_imu_data();
        convertImuData(*imu, *proto_imu);
    }
    
    if (gnss) {
        auto proto_gnss = packet.mutable_gnss_data();
        convertGnssData(*gnss, *proto_gnss);
    }
    
    // Convert camera data
    for (size_t i = 0; i < cameras.size(); ++i) {
        if (cameras[i]) {
            auto proto_camera = packet.add_camera_data();
            convertCameraData(*cameras[i], *proto_camera, static_cast<int>(i));
        }
    }
    
    return packet;
}

// Conversion utilities (simplified implementations)
void HybridSynchronizer::convertLidarData(const sensor_msgs::PointCloud2& ros_msg, LidarData& proto_msg) const {
    proto_msg.set_width(ros_msg.width);
    proto_msg.set_height(ros_msg.height);
    proto_msg.set_is_dense(ros_msg.is_dense);
    proto_msg.set_point_step(ros_msg.point_step);
    proto_msg.set_row_step(ros_msg.row_step);
    proto_msg.set_point_data(ros_msg.data.data(), ros_msg.data.size());
}

void HybridSynchronizer::convertImuData(const sensor_msgs::Imu& ros_msg, ImuData& proto_msg) const {
    auto linear_acc = proto_msg.mutable_linear_acceleration();
    linear_acc->set_x(ros_msg.linear_acceleration.x);
    linear_acc->set_y(ros_msg.linear_acceleration.y);
    linear_acc->set_z(ros_msg.linear_acceleration.z);
    
    auto angular_vel = proto_msg.mutable_angular_velocity();
    angular_vel->set_x(ros_msg.angular_velocity.x);
    angular_vel->set_y(ros_msg.angular_velocity.y);
    angular_vel->set_z(ros_msg.angular_velocity.z);
    
    auto orientation = proto_msg.mutable_orientation();
    orientation->set_x(ros_msg.orientation.x);
    orientation->set_y(ros_msg.orientation.y);
    orientation->set_z(ros_msg.orientation.z);
    orientation->set_w(ros_msg.orientation.w);
}

void HybridSynchronizer::convertGnssData(const sensor_msgs::NavSatFix& ros_msg, GnssData& proto_msg) const {
    proto_msg.set_latitude(ros_msg.latitude);
    proto_msg.set_longitude(ros_msg.longitude);
    proto_msg.set_altitude(ros_msg.altitude);
    proto_msg.set_status(ros_msg.status.status);
    proto_msg.set_service(ros_msg.status.service);
}

void HybridSynchronizer::convertCameraData(const sensor_msgs::CompressedImage& ros_msg, CameraData& proto_msg, int camera_id) const {
    proto_msg.set_camera_id(static_cast<uint32_t>(camera_id));
    proto_msg.set_encoding(ros_msg.format);
    proto_msg.set_data(ros_msg.data.data(), ros_msg.data.size());
    // Note: CompressedImage doesn't have width/height/step info
}

size_t HybridSynchronizer::estimatePacketSize(
    const sensor_msgs::PointCloud2::ConstPtr& lidar,
    const std::vector<sensor_msgs::CompressedImage::ConstPtr>& cameras) {
    
    size_t total_size = 0;
    
    // LiDAR data (typically largest)
    if (lidar) {
        total_size += lidar->data.size();
    }
    
    // Camera data (compressed images)
    for (const auto& camera : cameras) {
        if (camera) {
            total_size += camera->data.size();
        }
    }
    
    // Add overhead for protobuf serialization (~10-20%)
    total_size = static_cast<size_t>(total_size * 1.15);
    
    return total_size;
}

HybridSynchronizer::Metrics HybridSynchronizer::getMetrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return metrics_;
}

// DEBUG CALLBACKS - Individual message tracking
void HybridSynchronizer::debugLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    uint32_t count = debug_lidar_count_.fetch_add(1) + 1;
    ROS_DEBUG_THROTTLE(2.0, "LiDAR msg #%u: %u points, timestamp: %u.%09u",
                      count, msg->width * msg->height, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void HybridSynchronizer::debugImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    uint32_t count = debug_imu_count_.fetch_add(1) + 1;
    ROS_DEBUG_THROTTLE(2.0, "IMU msg #%u: acc(%.2f,%.2f,%.2f), timestamp: %u.%09u",
                      count, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                      msg->header.stamp.sec, msg->header.stamp.nsec);
}

void HybridSynchronizer::debugGnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    uint32_t count = debug_gnss_count_.fetch_add(1) + 1;
    ROS_DEBUG_THROTTLE(2.0, "GNSS msg #%u: lat=%.6f, lon=%.6f, timestamp: %u.%09u",
                      count, msg->latitude, msg->longitude, msg->header.stamp.sec, msg->header.stamp.nsec);
}

void HybridSynchronizer::debugCam0Callback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    uint32_t count = debug_cam0_count_.fetch_add(1) + 1;
    ROS_DEBUG_THROTTLE(2.0, "CAM0 msg #%u: %zu bytes, timestamp: %u.%09u",
                      count, msg->data.size(), msg->header.stamp.sec, msg->header.stamp.nsec);
}


} // namespace robot_communication