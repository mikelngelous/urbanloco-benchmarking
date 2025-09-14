#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <memory>
#include "sensor_data.pb.h"

namespace robot_communication {

/**
 * @brief Thread-safe queue for decoupling data synchronization from network communication
 * 
 * Implements the Producer-Consumer pattern with:
 * - Non-blocking enqueue for producers (HybridSynchronizer)
 * - Blocking dequeue for consumers (CommunicationManager)
 * - Configurable size limits and drop policies
 * - Performance metrics and monitoring
 */
class DataTransferQueue {
public:
    struct Config {
        size_t max_queue_size;              // Maximum packets in queue
        bool drop_oldest_on_full;           // Drop policy when full
        std::chrono::milliseconds timeout_ms; // Dequeue timeout
        bool enable_metrics;                // Enable performance tracking
        
        // Constructor with default values
        Config() : max_queue_size(1000), drop_oldest_on_full(true), 
                   timeout_ms(100), enable_metrics(true) {}
    };

    struct Metrics {
        std::atomic<uint64_t> packets_enqueued{0};
        std::atomic<uint64_t> packets_dequeued{0};
        std::atomic<uint64_t> packets_dropped{0};
        std::atomic<size_t> current_size{0};
        std::atomic<size_t> peak_size{0};
        std::chrono::steady_clock::time_point start_time;
        
        Metrics() : start_time(std::chrono::steady_clock::now()) {}
        
        // Move constructor
        Metrics(Metrics&& other) noexcept 
            : packets_enqueued(other.packets_enqueued.load()),
              packets_dequeued(other.packets_dequeued.load()),
              packets_dropped(other.packets_dropped.load()),
              current_size(other.current_size.load()),
              peak_size(other.peak_size.load()),
              start_time(std::move(other.start_time)) {}
        
        // Copy constructor (deleted)
        Metrics(const Metrics&) = delete;
        
        // Assignment operators (deleted)
        Metrics& operator=(const Metrics&) = delete;
        Metrics& operator=(Metrics&&) = delete;
        
        double getDropRate() const {
            uint64_t total = packets_enqueued.load();
            return total > 0 ? (double)packets_dropped.load() / total : 0.0;
        }
        
        double getThroughputHz() const {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
            return seconds > 0 ? (double)packets_dequeued.load() / seconds : 0.0;
        }
    };

    explicit DataTransferQueue(const Config& config = Config());
    ~DataTransferQueue();

    /**
     * @brief Enqueue a sensor data packet (non-blocking for producer)
     * @param packet Sensor data packet to enqueue
     * @return true if enqueued, false if dropped due to full queue
     */
    bool enqueue(const SensorDataPacket& packet);

    /**
     * @brief Dequeue a sensor data packet (blocking for consumer)
     * @param packet Output parameter to store dequeued packet
     * @param timeout_ms Maximum time to wait for data
     * @return true if packet retrieved, false if timeout or shutdown
     */
    bool dequeue(SensorDataPacket& packet, std::chrono::milliseconds timeout_ms = std::chrono::milliseconds::max());

    /**
     * @brief Get current queue size
     */
    size_t size() const;

    /**
     * @brief Check if queue is empty
     */
    bool empty() const;

    /**
     * @brief Get performance metrics
     */
    Metrics getMetrics() const {
        Metrics copy;
        copy.packets_enqueued.store(metrics_.packets_enqueued.load());
        copy.packets_dequeued.store(metrics_.packets_dequeued.load());
        copy.packets_dropped.store(metrics_.packets_dropped.load());
        copy.current_size.store(metrics_.current_size.load());
        copy.peak_size.store(metrics_.peak_size.load());
        copy.start_time = metrics_.start_time;
        return copy;
    }

    /**
     * @brief Shutdown the queue (unblocks all waiting consumers)
     */
    void shutdown();

    /**
     * @brief Clear all queued data
     */
    void clear();

private:
    Config config_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<SensorDataPacket> queue_;
    std::atomic<bool> shutdown_flag_{false};
    mutable Metrics metrics_;

    void updateMetrics();
    void dropOldestPackets(size_t count);
};

} // namespace robot_communication