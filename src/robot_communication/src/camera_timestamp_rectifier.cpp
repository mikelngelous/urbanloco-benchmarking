#include "robot_communication/camera_timestamp_rectifier.hpp"

namespace robot_communication {

CameraTimestampRectifier::CameraTimestampRectifier(ros::NodeHandle& nh, 
                                                   const std::string& input_topic,
                                                   const std::string& output_topic)
    : nh_(nh), input_topic_(input_topic), output_topic_(output_topic), message_count_(0) {
    
    recent_timestamps_.reserve(FPS_WINDOW_SIZE);
    
    // Dataset base time for UrbanLoco CA (Aug 29 2019 03:47:09)
    dataset_base_time_.sec = 1567043229;  // Base timestamp from rosbag info
    dataset_base_time_.nsec = 0;
    
    ROS_INFO("CameraTimestampRectifier initialized:");
    ROS_INFO("  Input topic: %s", input_topic_.c_str());
    ROS_INFO("  Output topic: %s", output_topic_.c_str());
    ROS_INFO("  Dataset base time: %.3f", dataset_base_time_.toSec());
}

void CameraTimestampRectifier::start() {
    // Subscribe to raw camera topic
    camera_sub_ = nh_.subscribe(input_topic_, 10, 
                               &CameraTimestampRectifier::cameraCallback, this);
    
    // Advertise rectified camera topic
    camera_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(output_topic_, 10);
    
    ROS_INFO("CameraTimestampRectifier started - converting to real-time timestamps");
}

void CameraTimestampRectifier::stop() {
    camera_sub_.shutdown();
    camera_pub_.shutdown();
    
    ROS_INFO("CameraTimestampRectifier stopped");
    ROS_INFO("  Total messages: %lu", stats_.messages_received);
    ROS_INFO("  Zero timestamps fixed: %lu", stats_.zero_timestamps_fixed);
    ROS_INFO("  Valid timestamps passed: %lu", stats_.valid_timestamps_passed);
    ROS_INFO("  Average FPS: %.2f", stats_.average_fps);
}

void CameraTimestampRectifier::cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    stats_.messages_received++;
    
    // Create a mutable copy of the message
    sensor_msgs::CompressedImage rectified_msg = *msg;
    
    // Get current real-time timestamp
    ros::Time current_time = ros::Time::now();
    
    // REAL-TIME APPROACH: Always use current ROS time
    // Datasets are only for testing - in production we sync to current time
    
    // Always assign current ROS time for real-time synchronization
    rectified_msg.header.stamp = ros::Time::now();
    
    if (msg->header.stamp.isZero() || msg->header.stamp.toSec() == 0) {
        stats_.zero_timestamps_fixed++;
        ROS_DEBUG_THROTTLE(1.0, "Fixed zero timestamp #%lu (real-time: %.3f)", 
                          stats_.messages_received, rectified_msg.header.stamp.toSec());
    } else {
        stats_.valid_timestamps_passed++;
        ROS_DEBUG_THROTTLE(1.0, "Updated to real-time timestamp #%lu (was: %.3f, now: %.3f)", 
                          stats_.messages_received, msg->header.stamp.toSec(), 
                          rectified_msg.header.stamp.toSec());
    }
    
    // Ensure frame_id is set (required for TF and visualization)
    if (rectified_msg.header.frame_id.empty()) {
        rectified_msg.header.frame_id = "camera_array_cam0";
    }
    
    // Update sequence number if it was zero
    if (rectified_msg.header.seq == 0) {
        rectified_msg.header.seq = stats_.messages_received;
    }
    
    // Track timing for FPS calculation
    recent_timestamps_.push_back(current_time);
    if (recent_timestamps_.size() > FPS_WINDOW_SIZE) {
        recent_timestamps_.erase(recent_timestamps_.begin());
    }
    
    // Calculate and update FPS
    stats_.average_fps = calculateFPS();
    stats_.last_message_time = current_time;
    
    // Track first message time
    if (first_message_time_.isZero()) {
        first_message_time_ = current_time;
        ROS_INFO("First camera message received at %f", current_time.toSec());
    }
    
    // Publish rectified message with real-time timestamp
    camera_pub_.publish(rectified_msg);
    
    // Log every N messages
    if (stats_.messages_received % 100 == 0) {
        ROS_INFO("Camera rectifier: %lu msgs processed, FPS: %.2f, Zero timestamps fixed: %lu (%.1f%%)",
                stats_.messages_received, 
                stats_.average_fps,
                stats_.zero_timestamps_fixed,
                (100.0 * stats_.zero_timestamps_fixed) / stats_.messages_received);
    }
}

double CameraTimestampRectifier::calculateFPS() {
    if (recent_timestamps_.size() < 2) {
        return 0.0;
    }
    
    // Calculate average interval between recent messages
    double total_interval = 0.0;
    for (size_t i = 1; i < recent_timestamps_.size(); ++i) {
        total_interval += (recent_timestamps_[i] - recent_timestamps_[i-1]).toSec();
    }
    
    double avg_interval = total_interval / (recent_timestamps_.size() - 1);
    
    // FPS is inverse of average interval
    return (avg_interval > 0) ? (1.0 / avg_interval) : 0.0;
}

} // namespace robot_communication