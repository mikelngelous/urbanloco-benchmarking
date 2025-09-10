#include "robot_communication/sensor_fusion.hpp"
#include <ros/ros.h>

namespace robot_communication {

bool SensorFusion::fuseSensorData(const std::vector<sensor_msgs::PointCloud2::ConstPtr>& lidar_msgs,
                                 const std::vector<sensor_msgs::Imu::ConstPtr>& imu_msgs,
                                 const std::vector<sensor_msgs::NavSatFix::ConstPtr>& gnss_msgs,
                                 SensorDataPacket& output) {
    // Basic fusion - just create packet structure
    output.set_sequence_id(++sequence_counter_);
    output.set_frame_id("robot_frame");
    
    auto timestamp = output.mutable_timestamp();
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    timestamp->set_seconds(seconds.count());
    timestamp->set_nanos(static_cast<int32_t>(nanos.count()));
    
    return true;
}

} // namespace robot_communication