#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/pass_through.h>

namespace robot_communication {

/**
 * @brief Camera Timestamp Rectifier for real-time timestamp assignment
 * 
 * This class handles camera messages with zero timestamps (common in datasets)
 * and assigns real-time timestamps for synchronization with other sensors.
 * 
 * In real-time systems, all timestamps should reflect current time, not
 * historical bag timestamps. This allows proper synchronization in live
 * robot operation scenarios.
 */
class CameraTimestampRectifier {
public:
    /**
     * @brief Constructor
     * @param nh Node handle for subscriptions and publications
     * @param input_topic Camera topic to subscribe to
     * @param output_topic Rectified camera topic to publish
     */
    CameraTimestampRectifier(ros::NodeHandle& nh, 
                           const std::string& input_topic,
                           const std::string& output_topic);
    
    /**
     * @brief Start the rectifier
     */
    void start();
    
    /**
     * @brief Stop the rectifier
     */
    void stop();
    
    /**
     * @brief Get statistics about processed messages
     */
    struct Statistics {
        uint64_t messages_received = 0;
        uint64_t zero_timestamps_fixed = 0;
        uint64_t valid_timestamps_passed = 0;
        double average_fps = 0.0;
        ros::Time last_message_time;
    };
    
    Statistics getStatistics() const { return stats_; }

private:
    /**
     * @brief Callback for incoming camera messages
     * @param msg Incoming compressed image message
     */
    void cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    
    ros::NodeHandle& nh_;
    std::string input_topic_;
    std::string output_topic_;
    
    ros::Subscriber camera_sub_;
    ros::Publisher camera_pub_;
    
    // Statistics tracking
    Statistics stats_;
    ros::Time first_message_time_;
    ros::Time last_message_time_;
    
    // For FPS calculation
    static constexpr int FPS_WINDOW_SIZE = 30;
    std::vector<ros::Time> recent_timestamps_;
    
    // Dataset synchronization
    ros::Time dataset_base_time_;
    uint64_t message_count_;
    
    /**
     * @brief Calculate current FPS based on recent messages
     */
    double calculateFPS();
};

} // namespace robot_communication