#pragma once

#include <vector>
#include <atomic>
#include <memory>

namespace robot_communication {

/**
 * @brief Lock-free circular buffer for high-performance sensor data storage
 * 
 * Thread-safe single producer, single consumer circular buffer optimized 
 * for real-time sensor data processing with minimal allocation overhead.
 */
template<typename T>
class CircularBuffer {
public:
    /**
     * @brief Constructor
     * @param capacity Maximum number of elements
     */
    explicit CircularBuffer(size_t capacity = 1000) 
        : capacity_(capacity + 1), buffer_(capacity_) {}
    
    /**
     * @brief Resize buffer (not thread-safe, call only when stopped)
     */
    void resize(size_t new_capacity) {
        capacity_ = new_capacity + 1;
        buffer_.resize(capacity_);
        head_.store(0);
        tail_.store(0);
    }
    
    /**
     * @brief Push element (thread-safe for single producer)
     * @param item Element to push
     * @return true if successful, false if buffer full
     */
    bool push(const T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (current_tail + 1) % capacity_;
        
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Buffer full
        }
        
        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }
    
    /**
     * @brief Push element with move semantics
     */
    bool push(T&& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (current_tail + 1) % capacity_;
        
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Buffer full
        }
        
        buffer_[current_tail] = std::move(item);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }
    
    /**
     * @brief Pop element (thread-safe for single consumer)
     * @param item Reference to store popped element
     * @return true if successful, false if buffer empty
     */
    bool pop(T& item) {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Buffer empty
        }
        
        item = std::move(buffer_[current_head]);
        head_.store((current_head + 1) % capacity_, std::memory_order_release);
        return true;
    }
    
    /**
     * @brief Peek at front element without removing it
     */
    bool front(T& item) const {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false; // Buffer empty
        }
        
        item = buffer_[current_head];
        return true;
    }
    
    /**
     * @brief Check if buffer is empty
     */
    bool empty() const {
        return head_.load(std::memory_order_acquire) == 
               tail_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief Check if buffer is full
     */
    bool full() const {
        const size_t current_tail = tail_.load(std::memory_order_acquire);
        const size_t next_tail = (current_tail + 1) % capacity_;
        return next_tail == head_.load(std::memory_order_acquire);
    }
    
    /**
     * @brief Get current size (approximate, not atomic)
     */
    size_t size() const {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        return (tail >= head) ? (tail - head) : (capacity_ - head + tail);
    }
    
    /**
     * @brief Get buffer capacity
     */
    size_t capacity() const {
        return capacity_ - 1; // -1 because we reserve one slot
    }
    
    /**
     * @brief Clear all elements (not thread-safe)
     */
    void clear() {
        head_.store(0);
        tail_.store(0);
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};

} // namespace robot_communication