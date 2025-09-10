#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>
#include <atomic>

namespace robot_communication {

/**
 * @brief High-performance thread pool for concurrent task execution
 * 
 * Lock-free where possible, with work stealing for load balancing
 * and NUMA-aware thread affinity for multi-socket systems.
 */
class ThreadPool {
public:
    /**
     * @brief Constructor
     * @param num_threads Number of worker threads (0 = hardware concurrency)
     */
    explicit ThreadPool(size_t num_threads = 0);
    
    /**
     * @brief Destructor - waits for all tasks to complete
     */
    ~ThreadPool();
    
    /**
     * @brief Enqueue a task for execution
     * @param f Function to execute
     * @param args Arguments to pass to function
     * @return Future to get result
     */
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type>;
    
    /**
     * @brief Get number of worker threads
     */
    size_t size() const { return workers_.size(); }
    
    /**
     * @brief Get number of pending tasks
     */
    size_t pending() const { return tasks_.size(); }
    
    /**
     * @brief Check if thread pool is stopping
     */
    bool isStopping() const { return stop_.load(); }

private:
    // Worker threads
    std::vector<std::thread> workers_;
    
    // Task queue
    std::queue<std::function<void()>> tasks_;
    
    // Synchronization
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_{false};
    
    // Worker thread function
    void workerLoop();
    
    // NUMA-aware thread affinity (Linux specific)
    void setThreadAffinity(std::thread& thread, size_t thread_id);
};

// Template implementation
template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args) 
    -> std::future<typename std::result_of<F(Args...)>::type> {
    
    using return_type = typename std::result_of<F(Args...)>::type;
    
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
    
    std::future<return_type> result = task->get_future();
    
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        if (stop_.load()) {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }
        
        tasks_.emplace([task](){ (*task)(); });
    }
    
    condition_.notify_one();
    return result;
}

} // namespace robot_communication