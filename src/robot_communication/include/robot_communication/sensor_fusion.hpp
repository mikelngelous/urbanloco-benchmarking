#pragma once

#include <vector>
#include <memory>
#include <atomic>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "sensor_data.pb.h"

namespace robot_communication {

/**
 * @brief Sensor fusion class for combining multi-modal sensor data
 */
class SensorFusion {
public:
    SensorFusion() = default;
    virtual ~SensorFusion() = default;
    
    /**
     * @brief Fuse sensor data from multiple sources into protobuf packet
     */
    virtual bool fuseSensorData(const std::vector<sensor_msgs::PointCloud2::ConstPtr>& lidar_msgs,
                               const std::vector<sensor_msgs::Imu::ConstPtr>& imu_msgs,
                               const std::vector<sensor_msgs::NavSatFix::ConstPtr>& gnss_msgs,
                               SensorDataPacket& output);

private:
    std::atomic<uint64_t> sequence_counter_{0};
};

} // namespace robot_communication