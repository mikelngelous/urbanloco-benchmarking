#include "robot_communication/data_transfer_queue.hpp"
#include <ros/ros.h>

namespace robot_communication {

DataTransferQueue::DataTransferQueue(const Config& config) 
    : config_(config) {
    ROS_INFO("DataTransferQueue initialized with max_size=%zu, drop_oldest=%s", 
             config_.max_queue_size, config_.drop_oldest_on_full ? "true" : "false");
}

DataTransferQueue::~DataTransferQueue() {
    shutdown();
    ROS_INFO("DataTransferQueue destroyed. Final metrics: enqueued=%lu, dequeued=%lu, dropped=%lu", 
             metrics_.packets_enqueued.load(), 
             metrics_.packets_dequeued.load(), 
             metrics_.packets_dropped.load());
}

bool DataTransferQueue::enqueue(const SensorDataPacket& packet) {
    if (shutdown_flag_.load()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Check if queue is full
        if (queue_.size() >= config_.max_queue_size) {
            if (config_.drop_oldest_on_full) {
                // Drop oldest packet to make room
                queue_.pop();
                metrics_.packets_dropped.fetch_add(1);
                ROS_WARN_THROTTLE(1.0, "DataTransferQueue full, dropped oldest packet. Queue size: %zu", queue_.size());
            } else {
                // Reject new packet
                metrics_.packets_dropped.fetch_add(1);
                ROS_WARN_THROTTLE(1.0, "DataTransferQueue full, rejected new packet. Queue size: %zu", queue_.size());
                return false;
            }
        }

        // Add new packet
        queue_.push(packet);
        metrics_.packets_enqueued.fetch_add(1);
        
        // Update size metrics
        size_t current_size = queue_.size();
        metrics_.current_size.store(current_size);
        
        size_t peak = metrics_.peak_size.load();
        while (current_size > peak && !metrics_.peak_size.compare_exchange_weak(peak, current_size)) {
            // CAS loop to update peak size
        }
    }

    // Notify waiting consumers
    cv_.notify_one();
    return true;
}

bool DataTransferQueue::dequeue(SensorDataPacket& packet, std::chrono::milliseconds timeout_ms) {
    if (shutdown_flag_.load()) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    
    // Wait for data or timeout
    bool has_data = cv_.wait_for(lock, timeout_ms, [this] {
        return !queue_.empty() || shutdown_flag_.load();
    });

    if (shutdown_flag_.load()) {
        return false;
    }

    if (!has_data || queue_.empty()) {
        return false; // Timeout
    }

    // Extract packet
    packet = std::move(queue_.front());
    queue_.pop();
    metrics_.packets_dequeued.fetch_add(1);
    metrics_.current_size.store(queue_.size());

    return true;
}

size_t DataTransferQueue::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

bool DataTransferQueue::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

void DataTransferQueue::shutdown() {
    shutdown_flag_.store(true);
    cv_.notify_all();
}

void DataTransferQueue::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<SensorDataPacket> empty;
    queue_.swap(empty);
    metrics_.current_size.store(0);
    ROS_INFO("DataTransferQueue cleared");
}

void DataTransferQueue::updateMetrics() {
    // Metrics are updated atomically during enqueue/dequeue
    // This method is reserved for future complex metric calculations
}

void DataTransferQueue::dropOldestPackets(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t dropped = 0;
    while (!queue_.empty() && dropped < count) {
        queue_.pop();
        dropped++;
    }
    metrics_.packets_dropped.fetch_add(dropped);
    metrics_.current_size.store(queue_.size());
    
    if (dropped > 0) {
        ROS_WARN("Dropped %zu oldest packets from DataTransferQueue", dropped);
    }
}

} // namespace robot_communication